// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice
#include "audio_msgs/msg/detail/audio__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `encoding`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
audio_msgs__msg__Audio__init(audio_msgs__msg__Audio * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    audio_msgs__msg__Audio__fini(msg);
    return false;
  }
  // seq_num
  // frames
  // channels
  // sample_rate
  // encoding
  if (!rosidl_runtime_c__String__init(&msg->encoding)) {
    audio_msgs__msg__Audio__fini(msg);
    return false;
  }
  // is_bigendian
  // layout
  // step
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    audio_msgs__msg__Audio__fini(msg);
    return false;
  }
  return true;
}

void
audio_msgs__msg__Audio__fini(audio_msgs__msg__Audio * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // seq_num
  // frames
  // channels
  // sample_rate
  // encoding
  rosidl_runtime_c__String__fini(&msg->encoding);
  // is_bigendian
  // layout
  // step
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
audio_msgs__msg__Audio__are_equal(const audio_msgs__msg__Audio * lhs, const audio_msgs__msg__Audio * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // seq_num
  if (lhs->seq_num != rhs->seq_num) {
    return false;
  }
  // frames
  if (lhs->frames != rhs->frames) {
    return false;
  }
  // channels
  if (lhs->channels != rhs->channels) {
    return false;
  }
  // sample_rate
  if (lhs->sample_rate != rhs->sample_rate) {
    return false;
  }
  // encoding
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->encoding), &(rhs->encoding)))
  {
    return false;
  }
  // is_bigendian
  if (lhs->is_bigendian != rhs->is_bigendian) {
    return false;
  }
  // layout
  if (lhs->layout != rhs->layout) {
    return false;
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
audio_msgs__msg__Audio__copy(
  const audio_msgs__msg__Audio * input,
  audio_msgs__msg__Audio * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // seq_num
  output->seq_num = input->seq_num;
  // frames
  output->frames = input->frames;
  // channels
  output->channels = input->channels;
  // sample_rate
  output->sample_rate = input->sample_rate;
  // encoding
  if (!rosidl_runtime_c__String__copy(
      &(input->encoding), &(output->encoding)))
  {
    return false;
  }
  // is_bigendian
  output->is_bigendian = input->is_bigendian;
  // layout
  output->layout = input->layout;
  // step
  output->step = input->step;
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

audio_msgs__msg__Audio *
audio_msgs__msg__Audio__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__Audio * msg = (audio_msgs__msg__Audio *)allocator.allocate(sizeof(audio_msgs__msg__Audio), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(audio_msgs__msg__Audio));
  bool success = audio_msgs__msg__Audio__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
audio_msgs__msg__Audio__destroy(audio_msgs__msg__Audio * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    audio_msgs__msg__Audio__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
audio_msgs__msg__Audio__Sequence__init(audio_msgs__msg__Audio__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__Audio * data = NULL;

  if (size) {
    data = (audio_msgs__msg__Audio *)allocator.zero_allocate(size, sizeof(audio_msgs__msg__Audio), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = audio_msgs__msg__Audio__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        audio_msgs__msg__Audio__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
audio_msgs__msg__Audio__Sequence__fini(audio_msgs__msg__Audio__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      audio_msgs__msg__Audio__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

audio_msgs__msg__Audio__Sequence *
audio_msgs__msg__Audio__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  audio_msgs__msg__Audio__Sequence * array = (audio_msgs__msg__Audio__Sequence *)allocator.allocate(sizeof(audio_msgs__msg__Audio__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = audio_msgs__msg__Audio__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
audio_msgs__msg__Audio__Sequence__destroy(audio_msgs__msg__Audio__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    audio_msgs__msg__Audio__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
audio_msgs__msg__Audio__Sequence__are_equal(const audio_msgs__msg__Audio__Sequence * lhs, const audio_msgs__msg__Audio__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!audio_msgs__msg__Audio__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
audio_msgs__msg__Audio__Sequence__copy(
  const audio_msgs__msg__Audio__Sequence * input,
  audio_msgs__msg__Audio__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(audio_msgs__msg__Audio);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    audio_msgs__msg__Audio * data =
      (audio_msgs__msg__Audio *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!audio_msgs__msg__Audio__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          audio_msgs__msg__Audio__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!audio_msgs__msg__Audio__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
