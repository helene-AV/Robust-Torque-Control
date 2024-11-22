#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RobustTorqueControl_DLLIMPORT __declspec(dllimport)
#  define RobustTorqueControl_DLLEXPORT __declspec(dllexport)
#  define RobustTorqueControl_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RobustTorqueControl_DLLIMPORT __attribute__((visibility("default")))
#    define RobustTorqueControl_DLLEXPORT __attribute__((visibility("default")))
#    define RobustTorqueControl_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RobustTorqueControl_DLLIMPORT
#    define RobustTorqueControl_DLLEXPORT
#    define RobustTorqueControl_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RobustTorqueControl_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RobustTorqueControl_DLLAPI
#  define RobustTorqueControl_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RobustTorqueControl_EXPORTS
#    define RobustTorqueControl_DLLAPI RobustTorqueControl_DLLEXPORT
#  else
#    define RobustTorqueControl_DLLAPI RobustTorqueControl_DLLIMPORT
#  endif // RobustTorqueControl_EXPORTS
#  define RobustTorqueControl_LOCAL RobustTorqueControl_DLLLOCAL
#endif // RobustTorqueControl_STATIC