// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from translation_node:srv/TestV1.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "translation_node/srv/detail/test_v1__struct.h"
#include "translation_node/srv/detail/test_v1__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace translation_node
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _TestV1_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestV1_Request_type_support_ids_t;

static const _TestV1_Request_type_support_ids_t _TestV1_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestV1_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestV1_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestV1_Request_type_support_symbol_names_t _TestV1_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV1_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Request)),
  }
};

typedef struct _TestV1_Request_type_support_data_t
{
  void * data[2];
} _TestV1_Request_type_support_data_t;

static _TestV1_Request_type_support_data_t _TestV1_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestV1_Request_message_typesupport_map = {
  2,
  "translation_node",
  &_TestV1_Request_message_typesupport_ids.typesupport_identifier[0],
  &_TestV1_Request_message_typesupport_symbol_names.symbol_name[0],
  &_TestV1_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestV1_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestV1_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace translation_node

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, translation_node, srv, TestV1_Request)() {
  return &::translation_node::srv::rosidl_typesupport_c::TestV1_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "translation_node/srv/detail/test_v1__struct.h"
// already included above
// #include "translation_node/srv/detail/test_v1__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace translation_node
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _TestV1_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestV1_Response_type_support_ids_t;

static const _TestV1_Response_type_support_ids_t _TestV1_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestV1_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestV1_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestV1_Response_type_support_symbol_names_t _TestV1_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV1_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1_Response)),
  }
};

typedef struct _TestV1_Response_type_support_data_t
{
  void * data[2];
} _TestV1_Response_type_support_data_t;

static _TestV1_Response_type_support_data_t _TestV1_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestV1_Response_message_typesupport_map = {
  2,
  "translation_node",
  &_TestV1_Response_message_typesupport_ids.typesupport_identifier[0],
  &_TestV1_Response_message_typesupport_symbol_names.symbol_name[0],
  &_TestV1_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestV1_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestV1_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace translation_node

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, translation_node, srv, TestV1_Response)() {
  return &::translation_node::srv::rosidl_typesupport_c::TestV1_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "translation_node/srv/detail/test_v1__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace translation_node
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _TestV1_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestV1_type_support_ids_t;

static const _TestV1_type_support_ids_t _TestV1_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestV1_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestV1_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestV1_type_support_symbol_names_t _TestV1_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, translation_node, srv, TestV1)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, translation_node, srv, TestV1)),
  }
};

typedef struct _TestV1_type_support_data_t
{
  void * data[2];
} _TestV1_type_support_data_t;

static _TestV1_type_support_data_t _TestV1_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestV1_service_typesupport_map = {
  2,
  "translation_node",
  &_TestV1_service_typesupport_ids.typesupport_identifier[0],
  &_TestV1_service_typesupport_symbol_names.symbol_name[0],
  &_TestV1_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t TestV1_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestV1_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace translation_node

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, translation_node, srv, TestV1)() {
  return &::translation_node::srv::rosidl_typesupport_c::TestV1_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
