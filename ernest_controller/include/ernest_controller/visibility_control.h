#ifndef ernest_CONTROLLER__VISIBILITY_CONTROL_H_
#define ernest_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ernest_CONTROLLER_EXPORT __attribute__((dllexport))
#define ernest_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define ernest_CONTROLLER_EXPORT __declspec(dllexport)
#define ernest_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef ernest_CONTROLLER_BUILDING_DLL
#define ernest_CONTROLLER_PUBLIC ernest_CONTROLLER_EXPORT
#else
#define ernest_CONTROLLER_PUBLIC ernest_CONTROLLER_IMPORT
#endif
#define ernest_CONTROLLER_PUBLIC_TYPE ernest_CONTROLLER_PUBLIC
#define ernest_CONTROLLER_LOCAL
#else
#define ernest_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define ernest_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define ernest_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define ernest_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define ernest_CONTROLLER_PUBLIC
#define ernest_CONTROLLER_LOCAL
#endif
#define ernest_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ernest_CONTROLLER__VISIBILITY_CONTROL_H_
