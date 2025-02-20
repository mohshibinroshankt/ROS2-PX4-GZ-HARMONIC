// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from translation_node:srv/TestV0.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V0__TRAITS_HPP_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V0__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "translation_node/srv/detail/test_v0__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace translation_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const TestV0_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request_a
  {
    out << "request_a: ";
    rosidl_generator_traits::value_to_yaml(msg.request_a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TestV0_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_a: ";
    rosidl_generator_traits::value_to_yaml(msg.request_a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TestV0_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace translation_node

namespace rosidl_generator_traits
{

[[deprecated("use translation_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const translation_node::srv::TestV0_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  translation_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use translation_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const translation_node::srv::TestV0_Request & msg)
{
  return translation_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<translation_node::srv::TestV0_Request>()
{
  return "translation_node::srv::TestV0_Request";
}

template<>
inline const char * name<translation_node::srv::TestV0_Request>()
{
  return "translation_node/srv/TestV0_Request";
}

template<>
struct has_fixed_size<translation_node::srv::TestV0_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<translation_node::srv::TestV0_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<translation_node::srv::TestV0_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace translation_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const TestV0_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response_a
  {
    out << "response_a: ";
    rosidl_generator_traits::value_to_yaml(msg.response_a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TestV0_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response_a: ";
    rosidl_generator_traits::value_to_yaml(msg.response_a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TestV0_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace translation_node

namespace rosidl_generator_traits
{

[[deprecated("use translation_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const translation_node::srv::TestV0_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  translation_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use translation_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const translation_node::srv::TestV0_Response & msg)
{
  return translation_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<translation_node::srv::TestV0_Response>()
{
  return "translation_node::srv::TestV0_Response";
}

template<>
inline const char * name<translation_node::srv::TestV0_Response>()
{
  return "translation_node/srv/TestV0_Response";
}

template<>
struct has_fixed_size<translation_node::srv::TestV0_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<translation_node::srv::TestV0_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<translation_node::srv::TestV0_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<translation_node::srv::TestV0>()
{
  return "translation_node::srv::TestV0";
}

template<>
inline const char * name<translation_node::srv::TestV0>()
{
  return "translation_node/srv/TestV0";
}

template<>
struct has_fixed_size<translation_node::srv::TestV0>
  : std::integral_constant<
    bool,
    has_fixed_size<translation_node::srv::TestV0_Request>::value &&
    has_fixed_size<translation_node::srv::TestV0_Response>::value
  >
{
};

template<>
struct has_bounded_size<translation_node::srv::TestV0>
  : std::integral_constant<
    bool,
    has_bounded_size<translation_node::srv::TestV0_Request>::value &&
    has_bounded_size<translation_node::srv::TestV0_Response>::value
  >
{
};

template<>
struct is_service<translation_node::srv::TestV0>
  : std::true_type
{
};

template<>
struct is_service_request<translation_node::srv::TestV0_Request>
  : std::true_type
{
};

template<>
struct is_service_response<translation_node::srv::TestV0_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V0__TRAITS_HPP_
