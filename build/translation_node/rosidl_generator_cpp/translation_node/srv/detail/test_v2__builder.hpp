// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V2__BUILDER_HPP_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V2__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "translation_node/srv/detail/test_v2__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace translation_node
{

namespace srv
{

namespace builder
{

class Init_TestV2_Request_request_b
{
public:
  explicit Init_TestV2_Request_request_b(::translation_node::srv::TestV2_Request & msg)
  : msg_(msg)
  {}
  ::translation_node::srv::TestV2_Request request_b(::translation_node::srv::TestV2_Request::_request_b_type arg)
  {
    msg_.request_b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::translation_node::srv::TestV2_Request msg_;
};

class Init_TestV2_Request_request_a
{
public:
  Init_TestV2_Request_request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestV2_Request_request_b request_a(::translation_node::srv::TestV2_Request::_request_a_type arg)
  {
    msg_.request_a = std::move(arg);
    return Init_TestV2_Request_request_b(msg_);
  }

private:
  ::translation_node::srv::TestV2_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::translation_node::srv::TestV2_Request>()
{
  return translation_node::srv::builder::Init_TestV2_Request_request_a();
}

}  // namespace translation_node


namespace translation_node
{

namespace srv
{

namespace builder
{

class Init_TestV2_Response_response_b
{
public:
  explicit Init_TestV2_Response_response_b(::translation_node::srv::TestV2_Response & msg)
  : msg_(msg)
  {}
  ::translation_node::srv::TestV2_Response response_b(::translation_node::srv::TestV2_Response::_response_b_type arg)
  {
    msg_.response_b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::translation_node::srv::TestV2_Response msg_;
};

class Init_TestV2_Response_response_a
{
public:
  Init_TestV2_Response_response_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestV2_Response_response_b response_a(::translation_node::srv::TestV2_Response::_response_a_type arg)
  {
    msg_.response_a = std::move(arg);
    return Init_TestV2_Response_response_b(msg_);
  }

private:
  ::translation_node::srv::TestV2_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::translation_node::srv::TestV2_Response>()
{
  return translation_node::srv::builder::Init_TestV2_Response_response_a();
}

}  // namespace translation_node

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V2__BUILDER_HPP_
