// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_HPP_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__translation_node__srv__TestV2_Request __attribute__((deprecated))
#else
# define DEPRECATED__translation_node__srv__TestV2_Request __declspec(deprecated)
#endif

namespace translation_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestV2_Request_
{
  using Type = TestV2_Request_<ContainerAllocator>;

  explicit TestV2_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_a = 0;
      this->request_b = 0ull;
    }
  }

  explicit TestV2_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_a = 0;
      this->request_b = 0ull;
    }
  }

  // field types and members
  using _request_a_type =
    uint8_t;
  _request_a_type request_a;
  using _request_b_type =
    uint64_t;
  _request_b_type request_b;

  // setters for named parameter idiom
  Type & set__request_a(
    const uint8_t & _arg)
  {
    this->request_a = _arg;
    return *this;
  }
  Type & set__request_b(
    const uint64_t & _arg)
  {
    this->request_b = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t MESSAGE_VERSION =
    2u;

  // pointer types
  using RawPtr =
    translation_node::srv::TestV2_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const translation_node::srv::TestV2_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      translation_node::srv::TestV2_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      translation_node::srv::TestV2_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__translation_node__srv__TestV2_Request
    std::shared_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__translation_node__srv__TestV2_Request
    std::shared_ptr<translation_node::srv::TestV2_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestV2_Request_ & other) const
  {
    if (this->request_a != other.request_a) {
      return false;
    }
    if (this->request_b != other.request_b) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestV2_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestV2_Request_

// alias to use template instance with default allocator
using TestV2_Request =
  translation_node::srv::TestV2_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t TestV2_Request_<ContainerAllocator>::MESSAGE_VERSION;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace translation_node


#ifndef _WIN32
# define DEPRECATED__translation_node__srv__TestV2_Response __attribute__((deprecated))
#else
# define DEPRECATED__translation_node__srv__TestV2_Response __declspec(deprecated)
#endif

namespace translation_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestV2_Response_
{
  using Type = TestV2_Response_<ContainerAllocator>;

  explicit TestV2_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response_a = 0;
      this->response_b = 0ull;
    }
  }

  explicit TestV2_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response_a = 0;
      this->response_b = 0ull;
    }
  }

  // field types and members
  using _response_a_type =
    uint16_t;
  _response_a_type response_a;
  using _response_b_type =
    uint64_t;
  _response_b_type response_b;

  // setters for named parameter idiom
  Type & set__response_a(
    const uint16_t & _arg)
  {
    this->response_a = _arg;
    return *this;
  }
  Type & set__response_b(
    const uint64_t & _arg)
  {
    this->response_b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    translation_node::srv::TestV2_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const translation_node::srv::TestV2_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      translation_node::srv::TestV2_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      translation_node::srv::TestV2_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__translation_node__srv__TestV2_Response
    std::shared_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__translation_node__srv__TestV2_Response
    std::shared_ptr<translation_node::srv::TestV2_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestV2_Response_ & other) const
  {
    if (this->response_a != other.response_a) {
      return false;
    }
    if (this->response_b != other.response_b) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestV2_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestV2_Response_

// alias to use template instance with default allocator
using TestV2_Response =
  translation_node::srv::TestV2_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace translation_node

namespace translation_node
{

namespace srv
{

struct TestV2
{
  using Request = translation_node::srv::TestV2_Request;
  using Response = translation_node::srv::TestV2_Response;
};

}  // namespace srv

}  // namespace translation_node

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_HPP_
