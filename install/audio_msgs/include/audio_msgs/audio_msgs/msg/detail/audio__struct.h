// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_H_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LAYOUT_INTERLEAVED'.
enum
{
  audio_msgs__msg__Audio__LAYOUT_INTERLEAVED = 0
};

/// Constant 'LAYOUT_NON_INTERLEAVED'.
enum
{
  audio_msgs__msg__Audio__LAYOUT_NON_INTERLEAVED = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'encoding'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Audio in the package audio_msgs.
/**
  * This message contains uncompressed audio
 */
typedef struct audio_msgs__msg__Audio
{
  /// Header timestamp should be acquisition time the beginning of the buffer
  /// Header frame_id should be meaningful to the location of the transducers
  std_msgs__msg__Header header;
  /// accumulator of number of frames that came before this message
  uint64_t seq_num;
  /// Number of sample-rate intervals in this buffer
  uint32_t frames;
  /// number of channels of audio
  uint32_t channels;
  /// the sample rate of the audio in Hz (number type following Gstreamer)
  /// Values for encoding should follow Image message OpenCV convention
  int32_t sample_rate;
  /// Encoding of samples, channel meaning, ordering, etc
  /// this field should be extended
  rosidl_runtime_c__String encoding;
  /// is this data bigendian?
  uint8_t is_bigendian;
  /// are audio samples interleaved into frames?
  uint8_t layout;
  /// audio frame size in bytes
  uint32_t step;
  /// actual matrix data, size is (step * frames)
  rosidl_runtime_c__uint8__Sequence data;
} audio_msgs__msg__Audio;

// Struct for a sequence of audio_msgs__msg__Audio.
typedef struct audio_msgs__msg__Audio__Sequence
{
  audio_msgs__msg__Audio * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_msgs__msg__Audio__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_H_
