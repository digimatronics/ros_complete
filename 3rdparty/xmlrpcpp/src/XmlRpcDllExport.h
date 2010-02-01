#if defined(WIN32)
  #if defined(ROS_STATIC)
    #define XMLRPC_EXPORT
  #elif defined(XmlRpc_EXPORTS)
    #define XMLRPC_EXPORT __declspec(dllexport)
  #else
    #define XMLRPC_EXPORT __declspec(dllimport)
  #endif
#else
  #define ROSLIB_EXPORT
#endif

