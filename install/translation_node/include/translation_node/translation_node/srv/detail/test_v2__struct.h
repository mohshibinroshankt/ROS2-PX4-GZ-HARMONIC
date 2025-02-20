// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_H_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_H_

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
  translation_node__srv__TestV2_Request__MESSAGE_VERSION = 2ul
};

/// Struct defined in srv/TestV2 in the package translation_node.
typedef struct translation_node__srv__TestV2_Request
{
  uint8_t request_a;
  uint64_t request_b;
} translation_node__srv__TestV2_Request;

// Struct for a sequence of translation_node__srv__TestV2_Request.
typedef struct translation_node__srv__TestV2_Request__Sequence
{
  translation_node__srv__TestV2_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} translation_node__srv__TestV2_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TestV2 in the package translation_node.
typedef struct translation_node__srv__TestV2_Response
{
  uint16_t response_a;
  uint64_t response_b;
} translation_node__srv__TestV2_Response;

// Struct for a sequence of translation_node__srv__TestV2_Response.
typedef struct translation_node__srv__TestV2_Response__Sequence
{
  translation_node__srv__TestV2_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} translation_node__srv__TestV2_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V2__STRUCT_H_
