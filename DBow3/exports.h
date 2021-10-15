
#ifndef __DBOW_CORE_TYPES_H__
#define __DBOW_CORE_TYPES_H__
 
#if !defined _CRT_SECURE_NO_DEPRECATE && _MSC_VER > 1300
#define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio 2005 warnings */
#endif

#if (defined WIN32 || defined _WIN32 || defined WINCE)   && defined DBOW_DSO_EXPORTS
    #define DBOW_API __declspec(dllexport)
    #pragma warning ( disable : 4251 ) //disable warning to templates with dll linkage.
    #pragma warning ( disable : 4290 ) //disable warning due to exception specifications.
    #pragma warning ( disable : 4996 ) //disable warning regarding unsafe vsprintf.
    #pragma warning ( disable : 4244 ) //disable warning convesions with lost of data.

#else
    #define DBOW_API
#endif


#define  DBOW_VERSION "3.0.0"
#endif
