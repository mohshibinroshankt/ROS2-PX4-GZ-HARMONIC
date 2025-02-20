// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from translation_node:srv/TestV2.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "translation_node/srv/detail/test_v2__struct.h"
#include "translation_node/srv/detail/test_v2__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool translation_node__srv__test_v2__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("translation_node.srv._test_v2.TestV2_Request", full_classname_dest, 44) == 0);
  }
  translation_node__srv__TestV2_Request * ros_message = _ros_message;
  {  // request_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_a");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->request_a = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // request_b
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_b");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->request_b = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * translation_node__srv__test_v2__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TestV2_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("translation_node.srv._test_v2");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TestV2_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  translation_node__srv__TestV2_Request * ros_message = (translation_node__srv__TestV2_Request *)raw_ros_message;
  {  // request_a
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->request_a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request_b
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->request_b);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_b", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "translation_node/srv/detail/test_v2__struct.h"
// already included above
// #include "translation_node/srv/detail/test_v2__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool translation_node__srv__test_v2__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("translation_node.srv._test_v2.TestV2_Response", full_classname_dest, 45) == 0);
  }
  translation_node__srv__TestV2_Response * ros_message = _ros_message;
  {  // response_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "response_a");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->response_a = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // response_b
    PyObject * field = PyObject_GetAttrString(_pymsg, "response_b");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->response_b = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * translation_node__srv__test_v2__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TestV2_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("translation_node.srv._test_v2");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TestV2_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  translation_node__srv__TestV2_Response * ros_message = (translation_node__srv__TestV2_Response *)raw_ros_message;
  {  // response_a
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->response_a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "response_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // response_b
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->response_b);
    {
      int rc = PyObject_SetAttrString(_pymessage, "response_b", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
