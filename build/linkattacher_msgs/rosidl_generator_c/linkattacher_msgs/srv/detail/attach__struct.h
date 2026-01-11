// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from linkattacher_msgs:srv/Attach.idl
// generated code does not contain a copyright notice

#ifndef LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_H_
#define LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Attach in the package linkattacher_msgs.
typedef struct linkattacher_msgs__srv__Attach_Request
{
  int32_t robot_id;
  bool flag;
} linkattacher_msgs__srv__Attach_Request;

// Struct for a sequence of linkattacher_msgs__srv__Attach_Request.
typedef struct linkattacher_msgs__srv__Attach_Request__Sequence
{
  linkattacher_msgs__srv__Attach_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} linkattacher_msgs__srv__Attach_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Attach in the package linkattacher_msgs.
typedef struct linkattacher_msgs__srv__Attach_Response
{
  bool success;
} linkattacher_msgs__srv__Attach_Response;

// Struct for a sequence of linkattacher_msgs__srv__Attach_Response.
typedef struct linkattacher_msgs__srv__Attach_Response__Sequence
{
  linkattacher_msgs__srv__Attach_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} linkattacher_msgs__srv__Attach_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_H_
