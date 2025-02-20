// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from translation_node:srv/TestV0.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V0__STRUCT_H_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V0__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MESSAGE_VERSION'.
enum
{
  translation_node__srv__TestV0_Request__MESSAGE_VERSION = 0ul
};

/// Struct defined in srv/TestV0 in the package translation_node.
typedef struct translation_node__srv__TestV0_Request
{
  uint8_t request_a;
} translation_node__srv__TestV0_Request;

// Struct for a sequence of translation_node__srv__TestV0_Request.
typedef struct translation_node__srv__TestV0_Request__Sequence
{
  translation_node__srv__TestV0_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} translation_node__srv__TestV0_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TestV0 in the package translation_node.
typedef struct translation_node__srv__TestV0_Response
{
  uint64_t response_a;
} translation_node__srv__TestV0_Response;

// Struct for a sequence of translation_node__srv__TestV0_Response.
typedef struct translation_node__srv__TestV0_Response__Sequence
{
  translation_node__srv__TestV0_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} translation_node__srv__TestV0_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V0__STRUCT_H_
