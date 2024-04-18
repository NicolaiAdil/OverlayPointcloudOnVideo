// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO__BUILDER_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_msgs/msg/detail/audio__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_msgs
{

namespace msg
{

namespace builder
{

class Init_Audio_data
{
public:
  explicit Init_Audio_data(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  ::audio_msgs::msg::Audio data(::audio_msgs::msg::Audio::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_step
{
public:
  explicit Init_Audio_step(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_data step(::audio_msgs::msg::Audio::_step_type arg)
  {
    msg_.step = std::move(arg);
    return Init_Audio_data(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_layout
{
public:
  explicit Init_Audio_layout(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_step layout(::audio_msgs::msg::Audio::_layout_type arg)
  {
    msg_.layout = std::move(arg);
    return Init_Audio_step(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_is_bigendian
{
public:
  explicit Init_Audio_is_bigendian(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_layout is_bigendian(::audio_msgs::msg::Audio::_is_bigendian_type arg)
  {
    msg_.is_bigendian = std::move(arg);
    return Init_Audio_layout(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_encoding
{
public:
  explicit Init_Audio_encoding(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_is_bigendian encoding(::audio_msgs::msg::Audio::_encoding_type arg)
  {
    msg_.encoding = std::move(arg);
    return Init_Audio_is_bigendian(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_sample_rate
{
public:
  explicit Init_Audio_sample_rate(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_encoding sample_rate(::audio_msgs::msg::Audio::_sample_rate_type arg)
  {
    msg_.sample_rate = std::move(arg);
    return Init_Audio_encoding(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_channels
{
public:
  explicit Init_Audio_channels(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_sample_rate channels(::audio_msgs::msg::Audio::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return Init_Audio_sample_rate(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_frames
{
public:
  explicit Init_Audio_frames(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_channels frames(::audio_msgs::msg::Audio::_frames_type arg)
  {
    msg_.frames = std::move(arg);
    return Init_Audio_channels(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_seq_num
{
public:
  explicit Init_Audio_seq_num(::audio_msgs::msg::Audio & msg)
  : msg_(msg)
  {}
  Init_Audio_frames seq_num(::audio_msgs::msg::Audio::_seq_num_type arg)
  {
    msg_.seq_num = std::move(arg);
    return Init_Audio_frames(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

class Init_Audio_header
{
public:
  Init_Audio_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Audio_seq_num header(::audio_msgs::msg::Audio::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Audio_seq_num(msg_);
  }

private:
  ::audio_msgs::msg::Audio msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_msgs::msg::Audio>()
{
  return audio_msgs::msg::builder::Init_Audio_header();
}

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO__BUILDER_HPP_
