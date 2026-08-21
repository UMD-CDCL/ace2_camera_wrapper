#ifndef ACE2_CAMERA_WRAPPER__VISIBILITY_CONTROL_H_
#define ACE2_CAMERA_WRAPPER__VISIBILITY_CONTROL_H_

// Standard ament visibility boilerplate, so the component library exports its
// symbols consistently on every platform.
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACE2_CAMERA_WRAPPER_EXPORT __attribute__ ((dllexport))
    #define ACE2_CAMERA_WRAPPER_IMPORT __attribute__ ((dllimport))
  #else
    #define ACE2_CAMERA_WRAPPER_EXPORT __declspec(dllexport)
    #define ACE2_CAMERA_WRAPPER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACE2_CAMERA_WRAPPER_BUILDING_LIBRARY
    #define ACE2_CAMERA_WRAPPER_PUBLIC ACE2_CAMERA_WRAPPER_EXPORT
  #else
    #define ACE2_CAMERA_WRAPPER_PUBLIC ACE2_CAMERA_WRAPPER_IMPORT
  #endif
#else
  #define ACE2_CAMERA_WRAPPER_EXPORT __attribute__ ((visibility("default")))
  #define ACE2_CAMERA_WRAPPER_IMPORT
  #define ACE2_CAMERA_WRAPPER_PUBLIC __attribute__ ((visibility("default")))
#endif

#endif  // ACE2_CAMERA_WRAPPER__VISIBILITY_CONTROL_H_
