#include "OptimizableTypes.h"

namespace ORB_SLAM2 {
    bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

        return os.good();
    }


    void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                     -z , 0.f, x, 0.f, 1.f, 0.f,
                     y ,  -x , 0.f, 0.f, 0.f, 1.f;

        float fx = pK.at<float>(0,0);
        float fy = pK.at<float>(1,1);
        Eigen::Matrix<double,2,3> proj_jac;
        proj_jac << fx/xyz_trans[2], 0.f, -1*fx*xyz_trans[0]/(xyz_trans[2]* xyz_trans[2]),
                    0.f, fy/xyz_trans[2], -1*fy*xyz_trans[1]/(xyz_trans[2]* xyz_trans[2]);

        _jacobianOplusXi = -proj_jac * SE3deriv;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::read(std::istream& is){

        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPoseToBody::write(std::ostream& os) const {

        return os.good();
    }

    void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T_lw(vi->estimate());
        Eigen::Vector3d X_l = T_lw.map(Xw);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

        double x_w = X_l[0];
        double y_w = X_l[1];
        double z_w = X_l[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z_w,   -y_w, 1.f, 0.f, 0.f,
                -z_w , 0.f, x_w, 0.f, 1.f, 0.f,
                y_w ,  -x_w , 0.f, 0.f, 0.f, 1.f;
        float fx = pK.at<float>(0,0);
        float fy = pK.at<float>(1,1);
        Eigen::Matrix<double,2,3> proj_jac;
        proj_jac << fx/X_r[2], 0.f, -1*fx*X_r[0]/(X_r[2]* X_r[2]),
                    0.f, fy/X_r[2], -1*fy*X_r[1]/(X_r[2]* X_r[2]);
        _jacobianOplusXi = -proj_jac * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZ::read(std::istream& is){
        return true;
    }

    bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {
        return os.good();
    }


    void EdgeSE3ProjectXYZ::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        float fx = pK.at<float>(0,0);
        float fy = pK.at<float>(1,1);
        Eigen::Matrix<double,2,3> proj_jac;
        proj_jac << fx/xyz_trans[2], 0.f, -1*fx*xyz_trans[0]/(xyz_trans[2]* xyz_trans[2]),
                    0.f, fy/xyz_trans[2], -1*fy*xyz_trans[1]/(xyz_trans[2]* xyz_trans[2]);

        _jacobianOplusXi =  -proj_jac * T.rotation().toRotationMatrix();

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = -proj_jac * SE3deriv;
    }

    EdgeSE3ProjectXYZToBody::EdgeSE3ProjectXYZToBody() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZToBody::read(std::istream& is){
        return true;
    }

    bool EdgeSE3ProjectXYZToBody::write(std::ostream& os) const {
        return os.good();
    }


    void EdgeSE3ProjectXYZToBody::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T_lw(vj->estimate());
        g2o::SE3Quat T_rw = mTrl * T_lw;
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d X_w = vi->estimate();
        Eigen::Vector3d X_l = T_lw.map(X_w);
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(X_w));
        float fx = pK.at<float>(0,0);
        float fy = pK.at<float>(1,1);
        Eigen::Matrix<double,2,3> proj_jac;
        proj_jac << fx/X_r[2], 0.f, -1*fx*X_r[0]/(X_r[2]* X_r[2]),
                    0.f, fy/X_r[2], -1*fy*X_r[1]/(X_r[2]* X_r[2]);

        _jacobianOplusXi =  -proj_jac * T_rw.rotation().toRotationMatrix();

        double x = X_l[0];
        double y = X_l[1];
        double z = X_l[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXj = -proj_jac * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }


    VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>()
    {
        _marginalized=false;
        _fix_scale = false;
    }

    bool VertexSim3Expmap::read(std::istream& is)
    {
        return true;
    }

    bool VertexSim3Expmap::write(std::ostream& os) const
    {
        return os.good();
    }

    EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() :
            g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeSim3ProjectXYZ::read(std::istream& is)
    {
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream& os) const
    {
        return os.good();
    }

    EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
            g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool EdgeInverseSim3ProjectXYZ::read(std::istream& is)
    {

        return true;
    }

    bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const
    {
        return os.good();
    }

}
