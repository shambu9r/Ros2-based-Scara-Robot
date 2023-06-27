#ifndef SCARA_JOINT_VELOCITY_CONTROLLER__VISIBILITY_CONTROL_H_
#define SCARA_JOINT_VELOCITY_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SCARA_JOINT_VELOCITY_CONTROLLER_EXPORT __attribute__((dllexport))
#define SCARA_JOINT_VELOCITY_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define SCARA_JOINT_VELOCITY_CONTROLLER_EXPORT __declspec(dllexport)
#define SCARA_JOINT_VELOCITY_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef SCARA_JOINT_VELOCITY_CONTROLLER_BUILDING_DLL
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC SCARA_JOINT_VELOCITY_CONTROLLER_EXPORT
#else
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC SCARA_JOINT_VELOCITY_CONTROLLER_IMPORT
#endif
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC_TYPE SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC
#define SCARA_JOINT_VELOCITY_CONTROLLER_LOCAL
#else
#define SCARA_JOINT_VELOCITY_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define SCARA_JOINT_VELOCITY_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define SCARA_JOINT_VELOCITY_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC
#define SCARA_JOINT_VELOCITY_CONTROLLER_LOCAL
#endif
#define SCARA_JOINT_VELOCITY_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // SCARA_JOINT_VELOCITY_CONTROLLER__VISIBILITY_CONTROL_H_
