// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice
#include "translation_node/srv/detail/test_v2__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
translation_node__srv__TestV2_Request__init(translation_node__srv__TestV2_Request * msg)
{
  if (!msg) {
    return false;
  }
  // request_a
  // request_b
  return true;
}

void
translation_node__srv__TestV2_Request__fini(translation_node__srv__TestV2_Request * msg)
{
  if (!msg) {
    return;
  }
  // request_a
  // request_b
}

bool
translation_node__srv__TestV2_Request__are_equal(const translation_node__srv__TestV2_Request * lhs, const translation_node__srv__TestV2_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request_a
  if (lhs->request_a != rhs->request_a) {
    return false;
  }
  // request_b
  if (lhs->request_b != rhs->request_b) {
    return false;
  }
  return true;
}

bool
translation_node__srv__TestV2_Request__copy(
  const translation_node__srv__TestV2_Request * input,
  translation_node__srv__TestV2_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // request_a
  output->request_a = input->request_a;
  // request_b
  output->request_b = input->request_b;
  return true;
}

translation_node__srv__TestV2_Request *
translation_node__srv__TestV2_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Request * msg = (translation_node__srv__TestV2_Request *)allocator.allocate(sizeof(translation_node__srv__TestV2_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(translation_node__srv__TestV2_Request));
  bool success = translation_node__srv__TestV2_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
translation_node__srv__TestV2_Request__destroy(translation_node__srv__TestV2_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    translation_node__srv__TestV2_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
translation_node__srv__TestV2_Request__Sequence__init(translation_node__srv__TestV2_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Request * data = NULL;

  if (size) {
    data = (translation_node__srv__TestV2_Request *)allocator.zero_allocate(size, sizeof(translation_node__srv__TestV2_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = translation_node__srv__TestV2_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        translation_node__srv__TestV2_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
translation_node__srv__TestV2_Request__Sequence__fini(translation_node__srv__TestV2_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      translation_node__srv__TestV2_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

translation_node__srv__TestV2_Request__Sequence *
translation_node__srv__TestV2_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Request__Sequence * array = (translation_node__srv__TestV2_Request__Sequence *)allocator.allocate(sizeof(translation_node__srv__TestV2_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = translation_node__srv__TestV2_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
translation_node__srv__TestV2_Request__Sequence__destroy(translation_node__srv__TestV2_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    translation_node__srv__TestV2_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
translation_node__srv__TestV2_Request__Sequence__are_equal(const translation_node__srv__TestV2_Request__Sequence * lhs, const translation_node__srv__TestV2_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!translation_node__srv__TestV2_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
translation_node__srv__TestV2_Request__Sequence__copy(
  const translation_node__srv__TestV2_Request__Sequence * input,
  translation_node__srv__TestV2_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(translation_node__srv__TestV2_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    translation_node__srv__TestV2_Request * data =
      (translation_node__srv__TestV2_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!translation_node__srv__TestV2_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          translation_node__srv__TestV2_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!translation_node__srv__TestV2_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
translation_node__srv__TestV2_Response__init(translation_node__srv__TestV2_Response * msg)
{
  if (!msg) {
    return false;
  }
  // response_a
  // response_b
  return true;
}

void
translation_node__srv__TestV2_Response__fini(translation_node__srv__TestV2_Response * msg)
{
  if (!msg) {
    return;
  }
  // response_a
  // response_b
}

bool
translation_node__srv__TestV2_Response__are_equal(const translation_node__srv__TestV2_Response * lhs, const translation_node__srv__TestV2_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // response_a
  if (lhs->response_a != rhs->response_a) {
    return false;
  }
  // response_b
  if (lhs->response_b != rhs->response_b) {
    return false;
  }
  return true;
}

bool
translation_node__srv__TestV2_Response__copy(
  const translation_node__srv__TestV2_Response * input,
  translation_node__srv__TestV2_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // response_a
  output->response_a = input->response_a;
  // response_b
  output->response_b = input->response_b;
  return true;
}

translation_node__srv__TestV2_Response *
translation_node__srv__TestV2_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Response * msg = (translation_node__srv__TestV2_Response *)allocator.allocate(sizeof(translation_node__srv__TestV2_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(translation_node__srv__TestV2_Response));
  bool success = translation_node__srv__TestV2_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
translation_node__srv__TestV2_Response__destroy(translation_node__srv__TestV2_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    translation_node__srv__TestV2_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
translation_node__srv__TestV2_Response__Sequence__init(translation_node__srv__TestV2_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Response * data = NULL;

  if (size) {
    data = (translation_node__srv__TestV2_Response *)allocator.zero_allocate(size, sizeof(translation_node__srv__TestV2_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = translation_node__srv__TestV2_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        translation_node__srv__TestV2_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
translation_node__srv__TestV2_Response__Sequence__fini(translation_node__srv__TestV2_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      translation_node__srv__TestV2_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

translation_node__srv__TestV2_Response__Sequence *
translation_node__srv__TestV2_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  translation_node__srv__TestV2_Response__Sequence * array = (translation_node__srv__TestV2_Response__Sequence *)allocator.allocate(sizeof(translation_node__srv__TestV2_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = translation_node__srv__TestV2_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
translation_node__srv__TestV2_Response__Sequence__destroy(translation_node__srv__TestV2_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    translation_node__srv__TestV2_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
translation_node__srv__TestV2_Response__Sequence__are_equal(const translation_node__srv__TestV2_Response__Sequence * lhs, const translation_node__srv__TestV2_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!translation_node__srv__TestV2_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
translation_node__srv__TestV2_Response__Sequence__copy(
  const translation_node__srv__TestV2_Response__Sequence * input,
  translation_node__srv__TestV2_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(translation_node__srv__TestV2_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    translation_node__srv__TestV2_Response * data =
      (translation_node__srv__TestV2_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!translation_node__srv__TestV2_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          translation_node__srv__TestV2_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!translation_node__srv__TestV2_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
