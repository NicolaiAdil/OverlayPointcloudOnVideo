// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "audio_msgs/msg/detail/audio__rosidl_typesupport_introspection_c.h"
#include "audio_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "audio_msgs/msg/detail/audio__functions.h"
#include "audio_msgs/msg/detail/audio__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `encoding`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  audio_msgs__msg__Audio__init(message_memory);
}

void audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_fini_function(void * message_memory)
{
  audio_msgs__msg__Audio__fini(message_memory);
}

size_t audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__size_function__Audio__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_const_function__Audio__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_function__Audio__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__fetch_function__Audio__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_const_function__Audio__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__assign_function__Audio__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_function__Audio__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__resize_function__Audio__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "seq_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, seq_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frames",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, frames),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, channels),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sample_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, sample_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "encoding",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, encoding),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_bigendian",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, is_bigendian),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "layout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, layout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "step",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, step),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(audio_msgs__msg__Audio, data),  // bytes offset in struct
    NULL,  // default value
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__size_function__Audio__data,  // size() function pointer
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_const_function__Audio__data,  // get_const(index) function pointer
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__get_function__Audio__data,  // get(index) function pointer
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__fetch_function__Audio__data,  // fetch(index, &value) function pointer
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__assign_function__Audio__data,  // assign(index, value) function pointer
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__resize_function__Audio__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_members = {
  "audio_msgs__msg",  // message namespace
  "Audio",  // message name
  10,  // number of fields
  sizeof(audio_msgs__msg__Audio),
  audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_member_array,  // message members
  audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_init_function,  // function to initialize message memory (memory has to be allocated)
  audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_type_support_handle = {
  0,
  &audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_audio_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, audio_msgs, msg, Audio)() {
  audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_type_support_handle.typesupport_identifier) {
    audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &audio_msgs__msg__Audio__rosidl_typesupport_introspection_c__Audio_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
