// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_msgs:msg/Audio.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_HPP_
#define AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__audio_msgs__msg__Audio __attribute__((deprecated))
#else
# define DEPRECATED__audio_msgs__msg__Audio __declspec(deprecated)
#endif

namespace audio_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Audio_
{
  using Type = Audio_<ContainerAllocator>;

  explicit Audio_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seq_num = 0ull;
      this->frames = 0ul;
      this->channels = 0ul;
      this->sample_rate = 0l;
      this->encoding = "";
      this->is_bigendian = 0;
      this->layout = 0;
      this->step = 0ul;
    }
  }

  explicit Audio_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    encoding(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seq_num = 0ull;
      this->frames = 0ul;
      this->channels = 0ul;
      this->sample_rate = 0l;
      this->encoding = "";
      this->is_bigendian = 0;
      this->layout = 0;
      this->step = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _seq_num_type =
    uint64_t;
  _seq_num_type seq_num;
  using _frames_type =
    uint32_t;
  _frames_type frames;
  using _channels_type =
    uint32_t;
  _channels_type channels;
  using _sample_rate_type =
    int32_t;
  _sample_rate_type sample_rate;
  using _encoding_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _encoding_type encoding;
  using _is_bigendian_type =
    uint8_t;
  _is_bigendian_type is_bigendian;
  using _layout_type =
    uint8_t;
  _layout_type layout;
  using _step_type =
    uint32_t;
  _step_type step;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__seq_num(
    const uint64_t & _arg)
  {
    this->seq_num = _arg;
    return *this;
  }
  Type & set__frames(
    const uint32_t & _arg)
  {
    this->frames = _arg;
    return *this;
  }
  Type & set__channels(
    const uint32_t & _arg)
  {
    this->channels = _arg;
    return *this;
  }
  Type & set__sample_rate(
    const int32_t & _arg)
  {
    this->sample_rate = _arg;
    return *this;
  }
  Type & set__encoding(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->encoding = _arg;
    return *this;
  }
  Type & set__is_bigendian(
    const uint8_t & _arg)
  {
    this->is_bigendian = _arg;
    return *this;
  }
  Type & set__layout(
    const uint8_t & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__step(
    const uint32_t & _arg)
  {
    this->step = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LAYOUT_INTERLEAVED =
    0u;
  static constexpr uint8_t LAYOUT_NON_INTERLEAVED =
    1u;

  // pointer types
  using RawPtr =
    audio_msgs::msg::Audio_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_msgs::msg::Audio_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_msgs::msg::Audio_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_msgs::msg::Audio_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::Audio_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::Audio_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_msgs::msg::Audio_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_msgs::msg::Audio_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_msgs::msg::Audio_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_msgs::msg::Audio_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_msgs__msg__Audio
    std::shared_ptr<audio_msgs::msg::Audio_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_msgs__msg__Audio
    std::shared_ptr<audio_msgs::msg::Audio_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Audio_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->seq_num != other.seq_num) {
      return false;
    }
    if (this->frames != other.frames) {
      return false;
    }
    if (this->channels != other.channels) {
      return false;
    }
    if (this->sample_rate != other.sample_rate) {
      return false;
    }
    if (this->encoding != other.encoding) {
      return false;
    }
    if (this->is_bigendian != other.is_bigendian) {
      return false;
    }
    if (this->layout != other.layout) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Audio_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Audio_

// alias to use template instance with default allocator
using Audio =
  audio_msgs::msg::Audio_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Audio_<ContainerAllocator>::LAYOUT_INTERLEAVED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Audio_<ContainerAllocator>::LAYOUT_NON_INTERLEAVED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace audio_msgs

#endif  // AUDIO_MSGS__MSG__DETAIL__AUDIO__STRUCT_HPP_
