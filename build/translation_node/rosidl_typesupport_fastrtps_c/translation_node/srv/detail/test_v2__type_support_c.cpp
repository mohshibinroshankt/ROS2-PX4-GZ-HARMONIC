// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice
#include "translation_node/srv/detail/test_v2__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "translation_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "translation_node/srv/detail/test_v2__struct.h"
#include "translation_node/srv/detail/test_v2__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TestV2_Request__ros_msg_type = translation_node__srv__TestV2_Request;

static bool _TestV2_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TestV2_Request__ros_msg_type * ros_message = static_cast<const _TestV2_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: request_a
  {
    cdr << ros_message->request_a;
  }

  // Field name: request_b
  {
    cdr << ros_message->request_b;
  }

  return true;
}

static bool _TestV2_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TestV2_Request__ros_msg_type * ros_message = static_cast<_TestV2_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: request_a
  {
    cdr >> ros_message->request_a;
  }

  // Field name: request_b
  {
    cdr >> ros_message->request_b;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_translation_node
size_t get_serialized_size_translation_node__srv__TestV2_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TestV2_Request__ros_msg_type * ros_message = static_cast<const _TestV2_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name request_a
  {
    size_t item_size = sizeof(ros_message->request_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name request_b
  {
    size_t item_size = sizeof(ros_message->request_b);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TestV2_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_translation_node__srv__TestV2_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_translation_node
size_t max_serialized_size_translation_node__srv__TestV2_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: request_a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: request_b
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = translation_node__srv__TestV2_Request;
    is_plain =
      (
      offsetof(DataType, request_b) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TestV2_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_translation_node__srv__TestV2_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TestV2_Request = {
  "translation_node::srv",
  "TestV2_Request",
  _TestV2_Request__cdr_serialize,
  _TestV2_Request__cdr_deserialize,
  _TestV2_Request__get_serialized_size,
  _TestV2_Request__max_serialized_size
};

static rosidl_message_type_support_t _TestV2_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TestV2_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV2_Request)() {
  return &_TestV2_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "translation_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "translation_node/srv/detail/test_v2__struct.h"
// already included above
// #include "translation_node/srv/detail/test_v2__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TestV2_Response__ros_msg_type = translation_node__srv__TestV2_Response;

static bool _TestV2_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TestV2_Response__ros_msg_type * ros_message = static_cast<const _TestV2_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: response_a
  {
    cdr << ros_message->response_a;
  }

  // Field name: response_b
  {
    cdr << ros_message->response_b;
  }

  return true;
}

static bool _TestV2_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TestV2_Response__ros_msg_type * ros_message = static_cast<_TestV2_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: response_a
  {
    cdr >> ros_message->response_a;
  }

  // Field name: response_b
  {
    cdr >> ros_message->response_b;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_translation_node
size_t get_serialized_size_translation_node__srv__TestV2_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TestV2_Response__ros_msg_type * ros_message = static_cast<const _TestV2_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name response_a
  {
    size_t item_size = sizeof(ros_message->response_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name response_b
  {
    size_t item_size = sizeof(ros_message->response_b);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TestV2_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_translation_node__srv__TestV2_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_translation_node
size_t max_serialized_size_translation_node__srv__TestV2_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: response_a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: response_b
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = translation_node__srv__TestV2_Response;
    is_plain =
      (
      offsetof(DataType, response_b) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TestV2_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_translation_node__srv__TestV2_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TestV2_Response = {
  "translation_node::srv",
  "TestV2_Response",
  _TestV2_Response__cdr_serialize,
  _TestV2_Response__cdr_deserialize,
  _TestV2_Response__get_serialized_size,
  _TestV2_Response__max_serialized_size
};

static rosidl_message_type_support_t _TestV2_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TestV2_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV2_Response)() {
  return &_TestV2_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "translation_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "translation_node/srv/test_v2.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t TestV2__callbacks = {
  "translation_node::srv",
  "TestV2",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV2_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV2_Response)(),
};

static rosidl_service_type_support_t TestV2__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &TestV2__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV2)() {
  return &TestV2__handle;
}

#if defined(__cplusplus)
}
#endif
