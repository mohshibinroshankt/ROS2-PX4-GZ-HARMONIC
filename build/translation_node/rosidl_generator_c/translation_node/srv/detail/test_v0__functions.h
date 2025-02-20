// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from translation_node:srv/TestV0.idl
// generated code does not contain a copyright notice

#ifndef TRANSLATION_NODE__SRV__DETAIL__TEST_V0__FUNCTIONS_H_
#define TRANSLATION_NODE__SRV__DETAIL__TEST_V0__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "translation_node/msg/rosidl_generator_c__visibility_control.h"

#include "translation_node/srv/detail/test_v0__struct.h"

/// Initialize srv/TestV0 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * translation_node__srv__TestV0_Request
 * )) before or use
 * translation_node__srv__TestV0_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__init(translation_node__srv__TestV0_Request * msg);

/// Finalize srv/TestV0 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Request__fini(translation_node__srv__TestV0_Request * msg);

/// Create srv/TestV0 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * translation_node__srv__TestV0_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
translation_node__srv__TestV0_Request *
translation_node__srv__TestV0_Request__create();

/// Destroy srv/TestV0 message.
/**
 * It calls
 * translation_node__srv__TestV0_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Request__destroy(translation_node__srv__TestV0_Request * msg);

/// Check for srv/TestV0 message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__are_equal(const translation_node__srv__TestV0_Request * lhs, const translation_node__srv__TestV0_Request * rhs);

/// Copy a srv/TestV0 message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__copy(
  const translation_node__srv__TestV0_Request * input,
  translation_node__srv__TestV0_Request * output);

/// Initialize array of srv/TestV0 messages.
/**
 * It allocates the memory for the number of elements and calls
 * translation_node__srv__TestV0_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__Sequence__init(translation_node__srv__TestV0_Request__Sequence * array, size_t size);

/// Finalize array of srv/TestV0 messages.
/**
 * It calls
 * translation_node__srv__TestV0_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Request__Sequence__fini(translation_node__srv__TestV0_Request__Sequence * array);

/// Create array of srv/TestV0 messages.
/**
 * It allocates the memory for the array and calls
 * translation_node__srv__TestV0_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
translation_node__srv__TestV0_Request__Sequence *
translation_node__srv__TestV0_Request__Sequence__create(size_t size);

/// Destroy array of srv/TestV0 messages.
/**
 * It calls
 * translation_node__srv__TestV0_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Request__Sequence__destroy(translation_node__srv__TestV0_Request__Sequence * array);

/// Check for srv/TestV0 message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__Sequence__are_equal(const translation_node__srv__TestV0_Request__Sequence * lhs, const translation_node__srv__TestV0_Request__Sequence * rhs);

/// Copy an array of srv/TestV0 messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Request__Sequence__copy(
  const translation_node__srv__TestV0_Request__Sequence * input,
  translation_node__srv__TestV0_Request__Sequence * output);

/// Initialize srv/TestV0 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * translation_node__srv__TestV0_Response
 * )) before or use
 * translation_node__srv__TestV0_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__init(translation_node__srv__TestV0_Response * msg);

/// Finalize srv/TestV0 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Response__fini(translation_node__srv__TestV0_Response * msg);

/// Create srv/TestV0 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * translation_node__srv__TestV0_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
translation_node__srv__TestV0_Response *
translation_node__srv__TestV0_Response__create();

/// Destroy srv/TestV0 message.
/**
 * It calls
 * translation_node__srv__TestV0_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Response__destroy(translation_node__srv__TestV0_Response * msg);

/// Check for srv/TestV0 message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__are_equal(const translation_node__srv__TestV0_Response * lhs, const translation_node__srv__TestV0_Response * rhs);

/// Copy a srv/TestV0 message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__copy(
  const translation_node__srv__TestV0_Response * input,
  translation_node__srv__TestV0_Response * output);

/// Initialize array of srv/TestV0 messages.
/**
 * It allocates the memory for the number of elements and calls
 * translation_node__srv__TestV0_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__Sequence__init(translation_node__srv__TestV0_Response__Sequence * array, size_t size);

/// Finalize array of srv/TestV0 messages.
/**
 * It calls
 * translation_node__srv__TestV0_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Response__Sequence__fini(translation_node__srv__TestV0_Response__Sequence * array);

/// Create array of srv/TestV0 messages.
/**
 * It allocates the memory for the array and calls
 * translation_node__srv__TestV0_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
translation_node__srv__TestV0_Response__Sequence *
translation_node__srv__TestV0_Response__Sequence__create(size_t size);

/// Destroy array of srv/TestV0 messages.
/**
 * It calls
 * translation_node__srv__TestV0_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
void
translation_node__srv__TestV0_Response__Sequence__destroy(translation_node__srv__TestV0_Response__Sequence * array);

/// Check for srv/TestV0 message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__Sequence__are_equal(const translation_node__srv__TestV0_Response__Sequence * lhs, const translation_node__srv__TestV0_Response__Sequence * rhs);

/// Copy an array of srv/TestV0 messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_translation_node
bool
translation_node__srv__TestV0_Response__Sequence__copy(
  const translation_node__srv__TestV0_Response__Sequence * input,
  translation_node__srv__TestV0_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TRANSLATION_NODE__SRV__DETAIL__TEST_V0__FUNCTIONS_H_
