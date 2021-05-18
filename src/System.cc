

//主进程的实现文件

//包含了一些自建库
#include "System.h"
#include "Converter.h"		// TODO 目前还不是很明白这个是做什么的
//包含共有库
#include <thread>					//多线程
#include <pangolin/pangolin.h>		//可视化界面
#include <iomanip>					//主要是对cin,cout之类的一些操纵运算子
#include <unistd.h>
namespace ORB_SLAM3
{

//系统的构造函数，将会启动其他的线程
System::System(const string &strVocFile,					//词典文件路径
			   const string &strSettingsFile,				//配置文件路径
			   const eSensor sensor,						//传感器类型
               const bool bUseViewer):						//是否使用可视化界面
					 mSensor(sensor), 							//初始化传感器类型
					 mpViewer(static_cast<Viewer*>(NULL)),		//空。。。对象指针？  TODO 
					 mbReset(false),							//无复位标志
					 mbResetActiveMap(false),
					 mbActivateLocalizationMode(false),			//没有这个模式转换标志
        			 mbDeactivateLocalizationMode(false)		//没有这个模式转换标志
{
    // Output welcome message

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    //如果打开失败，就输出调试信息
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);

    }

    //建立一个新的ORB字典
    mpVocabulary = new ORBVocabulary();
    //获取字典加载状态
//    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    //如果加载失败，就输出调试信息
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }


    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas
    //mpMap = new Map();
    mpAtlas = new Atlas(0);
    //----

    if (mSensor==IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)

    mpTracker = new Tracking(this, 
                            mpVocabulary, 
                            mpFrameDrawer, 
                            mpMapDrawer,
                             mpAtlas, 
							 mpKeyFrameDatabase, 
							 strSettingsFile, 
							 mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpAtlas,
									mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,
								mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpAtlas, 
									mpKeyFrameDatabase, 
									mpVocabulary, 
									mSensor!=MONOCULAR); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, 
									mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
    	//如果指定了，程序的运行过程中需要运行可视化部分
    	//新建viewer
        mpViewer = new Viewer(this, 			//又是这个
        					  mpFrameDrawer,	//帧绘制器
        					  mpMapDrawer,		//地图绘制器
        					  mpTracker,		//追踪器
        					  strSettingsFile);	//配置文件的访问路径
#ifndef __APPLE__
        //新建viewer线程
        mptViewer = new thread(&Viewer::Run, mpViewer);
#endif
        //给运动追踪器设置其查看器
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    //设置进程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas)
{

    // Check mode change
    {
        // 独占锁，主要是为了mbActivateLocalizationMode和mbDeactivateLocalizationMode不会发生混乱
        unique_lock<mutex> lock(mMutexMode);
        // mbActivateLocalizationMode为true会关闭局部地图线程
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }


            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);
    //获取相机位姿的估计结果
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

//激活定位模式
void System::ActivateLocalizationMode()
{
	//上锁
    unique_lock<mutex> lock(mMutexMode);
    //设置标志
    mbActivateLocalizationMode = true;
}

//取消定位模式
void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

//判断是否地图有较大的改变
bool System::MapChanged()
{
    static int n=0;
    //其实整个函数功能实现的重点还是在这个GetLastBigChangeIdx函数上
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::StopViewer()
{
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

}

//退出
void System::Shutdown()
{
	//对局部建图线程和回环检测线程发送终止请求
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    StopViewer();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || 
    	  !mpLoopCloser->isFinished()  || 
    	   mpLoopCloser->isRunningGBA())			
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

#ifdef __APPLE__
void System::StartViewer()
{
    if (mpViewer)
        mpViewer->Run();
}

#endif
} //namespace ORB_SLAM
