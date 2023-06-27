#ifndef SCARA_HARDWARE__VISIBILITY_CONTROL_H_
#define SCARA_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SCARA_HARDWARE_EXPORT __attribute__((dllexport))
#define SCARA_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define SCARA_HARDWARE_EXPORT __declspec(dllexport)
#define SCARA_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef SCARA_HARDWARE_BUILDING_DLL
#define SCARA_HARDWARE_PUBLIC SCARA_HARDWARE_EXPORT
#else
#define SCARA_HARDWARE_PUBLIC SCARA_HARDWARE_IMPORT
#endif
#define SCARA_HARDWARE_PUBLIC_TYPE SCARA_HARDWARE_PUBLIC
#define SCARA_HARDWARE_LOCAL
#else
#define SCARA_HARDWARE_EXPORT __attribute__((visibility("default")))
#define SCARA_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define SCARA_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define SCARA_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define SCARA_HARDWARE_PUBLIC
#define SCARA_HARDWARE_LOCAL
#endif
#define SCARA_HARDWARE_PUBLIC_TYPE
#endif

#endif  // SCARA_HARDWARE__VISIBILITY_CONTROL_H_
