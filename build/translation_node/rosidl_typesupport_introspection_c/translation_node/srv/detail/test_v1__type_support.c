// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from translation_node:srv/TestV1.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "translation_node/srv/detail/test_v1__rosidl_typesupport_introspection_c.h"
#include "translation_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "translation_node/srv/detail/test_v1__functions.h"
#include "translation_node/srv/detail/test_v1__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  translation_node__srv__TestV1_Request__init(message_memory);
}

void translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_fini_function(void * message_memory)
{
  translation_node__srv__TestV1_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_member_array[1] = {
  {
    "request_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(translation_node__srv__TestV1_Request, request_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_members = {
  "translation_node__srv",  // message namespace
  "TestV1_Request",  // message name
  1,  // number of fields
  sizeof(translation_node__srv__TestV1_Request),
  translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_member_array,  // message members
  translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_type_support_handle = {
  0,
  &translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_translation_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Request)() {
  if (!translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_type_support_handle.typesupport_identifier) {
    translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &translation_node__srv__TestV1_Request__rosidl_typesupport_introspection_c__TestV1_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "translation_node/srv/detail/test_v1__rosidl_typesupport_introspection_c.h"
// already included above
// #include "translation_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "translation_node/srv/detail/test_v1__functions.h"
// already included above
// #include "translation_node/srv/detail/test_v1__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  translation_node__srv__TestV1_Response__init(message_memory);
}

void translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_fini_function(void * message_memory)
{
  translation_node__srv__TestV1_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_member_array[1] = {
  {
    "response_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(translation_node__srv__TestV1_Response, response_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_members = {
  "translation_node__srv",  // message namespace
  "TestV1_Response",  // message name
  1,  // number of fields
  sizeof(translation_node__srv__TestV1_Response),
  translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_member_array,  // message members
  translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_type_support_handle = {
  0,
  &translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_translation_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Response)() {
  if (!translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_type_support_handle.typesupport_identifier) {
    translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &translation_node__srv__TestV1_Response__rosidl_typesupport_introspection_c__TestV1_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "translation_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "translation_node/srv/detail/test_v1__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_members = {
  "translation_node__srv",  // service namespace
  "TestV1",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_Request_message_type_support_handle,
  NULL  // response message
  // translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_Response_message_type_support_handle
};

static rosidl_service_type_support_t translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_type_support_handle = {
  0,
  &translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_translation_node
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1)() {
  if (!translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_type_support_handle.typesupport_identifier) {
    translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Response)()->data;
  }

  return &translation_node__srv__detail__test_v1__rosidl_typesupport_introspection_c__TestV1_service_type_support_handle;
}
