// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from linkattacher_msgs:srv/Attach.idl
// generated code does not contain a copyright notice
#include "linkattacher_msgs/srv/detail/attach__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
linkattacher_msgs__srv__Attach_Request__init(linkattacher_msgs__srv__Attach_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // flag
  return true;
}

void
linkattacher_msgs__srv__Attach_Request__fini(linkattacher_msgs__srv__Attach_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // flag
}

bool
linkattacher_msgs__srv__Attach_Request__are_equal(const linkattacher_msgs__srv__Attach_Request * lhs, const linkattacher_msgs__srv__Attach_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // flag
  if (lhs->flag != rhs->flag) {
    return false;
  }
  return true;
}

bool
linkattacher_msgs__srv__Attach_Request__copy(
  const linkattacher_msgs__srv__Attach_Request * input,
  linkattacher_msgs__srv__Attach_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // flag
  output->flag = input->flag;
  return true;
}

linkattacher_msgs__srv__Attach_Request *
linkattacher_msgs__srv__Attach_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Request * msg = (linkattacher_msgs__srv__Attach_Request *)allocator.allocate(sizeof(linkattacher_msgs__srv__Attach_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(linkattacher_msgs__srv__Attach_Request));
  bool success = linkattacher_msgs__srv__Attach_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
linkattacher_msgs__srv__Attach_Request__destroy(linkattacher_msgs__srv__Attach_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    linkattacher_msgs__srv__Attach_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
linkattacher_msgs__srv__Attach_Request__Sequence__init(linkattacher_msgs__srv__Attach_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Request * data = NULL;

  if (size) {
    data = (linkattacher_msgs__srv__Attach_Request *)allocator.zero_allocate(size, sizeof(linkattacher_msgs__srv__Attach_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = linkattacher_msgs__srv__Attach_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        linkattacher_msgs__srv__Attach_Request__fini(&data[i - 1]);
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
linkattacher_msgs__srv__Attach_Request__Sequence__fini(linkattacher_msgs__srv__Attach_Request__Sequence * array)
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
      linkattacher_msgs__srv__Attach_Request__fini(&array->data[i]);
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

linkattacher_msgs__srv__Attach_Request__Sequence *
linkattacher_msgs__srv__Attach_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Request__Sequence * array = (linkattacher_msgs__srv__Attach_Request__Sequence *)allocator.allocate(sizeof(linkattacher_msgs__srv__Attach_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = linkattacher_msgs__srv__Attach_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
linkattacher_msgs__srv__Attach_Request__Sequence__destroy(linkattacher_msgs__srv__Attach_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    linkattacher_msgs__srv__Attach_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
linkattacher_msgs__srv__Attach_Request__Sequence__are_equal(const linkattacher_msgs__srv__Attach_Request__Sequence * lhs, const linkattacher_msgs__srv__Attach_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!linkattacher_msgs__srv__Attach_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
linkattacher_msgs__srv__Attach_Request__Sequence__copy(
  const linkattacher_msgs__srv__Attach_Request__Sequence * input,
  linkattacher_msgs__srv__Attach_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(linkattacher_msgs__srv__Attach_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    linkattacher_msgs__srv__Attach_Request * data =
      (linkattacher_msgs__srv__Attach_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!linkattacher_msgs__srv__Attach_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          linkattacher_msgs__srv__Attach_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!linkattacher_msgs__srv__Attach_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
linkattacher_msgs__srv__Attach_Response__init(linkattacher_msgs__srv__Attach_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
linkattacher_msgs__srv__Attach_Response__fini(linkattacher_msgs__srv__Attach_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
linkattacher_msgs__srv__Attach_Response__are_equal(const linkattacher_msgs__srv__Attach_Response * lhs, const linkattacher_msgs__srv__Attach_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
linkattacher_msgs__srv__Attach_Response__copy(
  const linkattacher_msgs__srv__Attach_Response * input,
  linkattacher_msgs__srv__Attach_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

linkattacher_msgs__srv__Attach_Response *
linkattacher_msgs__srv__Attach_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Response * msg = (linkattacher_msgs__srv__Attach_Response *)allocator.allocate(sizeof(linkattacher_msgs__srv__Attach_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(linkattacher_msgs__srv__Attach_Response));
  bool success = linkattacher_msgs__srv__Attach_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
linkattacher_msgs__srv__Attach_Response__destroy(linkattacher_msgs__srv__Attach_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    linkattacher_msgs__srv__Attach_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
linkattacher_msgs__srv__Attach_Response__Sequence__init(linkattacher_msgs__srv__Attach_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Response * data = NULL;

  if (size) {
    data = (linkattacher_msgs__srv__Attach_Response *)allocator.zero_allocate(size, sizeof(linkattacher_msgs__srv__Attach_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = linkattacher_msgs__srv__Attach_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        linkattacher_msgs__srv__Attach_Response__fini(&data[i - 1]);
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
linkattacher_msgs__srv__Attach_Response__Sequence__fini(linkattacher_msgs__srv__Attach_Response__Sequence * array)
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
      linkattacher_msgs__srv__Attach_Response__fini(&array->data[i]);
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

linkattacher_msgs__srv__Attach_Response__Sequence *
linkattacher_msgs__srv__Attach_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linkattacher_msgs__srv__Attach_Response__Sequence * array = (linkattacher_msgs__srv__Attach_Response__Sequence *)allocator.allocate(sizeof(linkattacher_msgs__srv__Attach_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = linkattacher_msgs__srv__Attach_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
linkattacher_msgs__srv__Attach_Response__Sequence__destroy(linkattacher_msgs__srv__Attach_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    linkattacher_msgs__srv__Attach_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
linkattacher_msgs__srv__Attach_Response__Sequence__are_equal(const linkattacher_msgs__srv__Attach_Response__Sequence * lhs, const linkattacher_msgs__srv__Attach_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!linkattacher_msgs__srv__Attach_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
linkattacher_msgs__srv__Attach_Response__Sequence__copy(
  const linkattacher_msgs__srv__Attach_Response__Sequence * input,
  linkattacher_msgs__srv__Attach_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(linkattacher_msgs__srv__Attach_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    linkattacher_msgs__srv__Attach_Response * data =
      (linkattacher_msgs__srv__Attach_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!linkattacher_msgs__srv__Attach_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          linkattacher_msgs__srv__Attach_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!linkattacher_msgs__srv__Attach_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
