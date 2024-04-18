// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO__TRAITS_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "audio_msgs/msg/detail/audio__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace audio_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Audio & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: seq_num
  {
    out << "seq_num: ";
    rosidl_generator_traits::value_to_yaml(msg.seq_num, out);
    out << ", ";
  }

  // member: frames
  {
    out << "frames: ";
    rosidl_generator_traits::value_to_yaml(msg.frames, out);
    out << ", ";
  }

  // member: channels
  {
    out << "channels: ";
    rosidl_generator_traits::value_to_yaml(msg.channels, out);
    out << ", ";
  }

  // member: sample_rate
  {
    out << "sample_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_rate, out);
    out << ", ";
  }

  // member: encoding
  {
    out << "encoding: ";
    rosidl_generator_traits::value_to_yaml(msg.encoding, out);
    out << ", ";
  }

  // member: is_bigendian
  {
    out << "is_bigendian: ";
    rosidl_generator_traits::value_to_yaml(msg.is_bigendian, out);
    out << ", ";
  }

  // member: layout
  {
    out << "layout: ";
    rosidl_generator_traits::value_to_yaml(msg.layout, out);
    out << ", ";
  }

  // member: step
  {
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Audio & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: seq_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq_num: ";
    rosidl_generator_traits::value_to_yaml(msg.seq_num, out);
    out << "\n";
  }

  // member: frames
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frames: ";
    rosidl_generator_traits::value_to_yaml(msg.frames, out);
    out << "\n";
  }

  // member: channels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channels: ";
    rosidl_generator_traits::value_to_yaml(msg.channels, out);
    out << "\n";
  }

  // member: sample_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_rate, out);
    out << "\n";
  }

  // member: encoding
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "encoding: ";
    rosidl_generator_traits::value_to_yaml(msg.encoding, out);
    out << "\n";
  }

  // member: is_bigendian
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_bigendian: ";
    rosidl_generator_traits::value_to_yaml(msg.is_bigendian, out);
    out << "\n";
  }

  // member: layout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "layout: ";
    rosidl_generator_traits::value_to_yaml(msg.layout, out);
    out << "\n";
  }

  // member: step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Audio & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace audio_msgs

namespace rosidl_generator_traits
{

[[deprecated("use audio_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const audio_msgs::msg::Audio & msg,
  std::ostream & out, size_t indentation = 0)
{
  audio_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use audio_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const audio_msgs::msg::Audio & msg)
{
  return audio_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<audio_msgs::msg::Audio>()
{
  return "audio_msgs::msg::Audio";
}

template<>
inline const char * name<audio_msgs::msg::Audio>()
{
  return "audio_msgs/msg/Audio";
}

template<>
struct has_fixed_size<audio_msgs::msg::Audio>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<audio_msgs::msg::Audio>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<audio_msgs::msg::Audio>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO__TRAITS_HPP_
