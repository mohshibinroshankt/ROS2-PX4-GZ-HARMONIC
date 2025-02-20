# generated from rosidl_generator_py/resource/_idl.py.em
# with input from translation_node:srv/TestV2.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TestV2_Request(type):
    """Metaclass of message 'TestV2_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MESSAGE_VERSION': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('translation_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'translation_node.srv.TestV2_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__test_v2__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__test_v2__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__test_v2__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__test_v2__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__test_v2__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MESSAGE_VERSION': cls.__constants['MESSAGE_VERSION'],
        }

    @property
    def MESSAGE_VERSION(self):
        """Message constant 'MESSAGE_VERSION'."""
        return Metaclass_TestV2_Request.__constants['MESSAGE_VERSION']


class TestV2_Request(metaclass=Metaclass_TestV2_Request):
    """
    Message class 'TestV2_Request'.

    Constants:
      MESSAGE_VERSION
    """

    __slots__ = [
        '_request_a',
        '_request_b',
    ]

    _fields_and_field_types = {
        'request_a': 'uint8',
        'request_b': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.request_a = kwargs.get('request_a', int())
        self.request_b = kwargs.get('request_b', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.request_a != other.request_a:
            return False
        if self.request_b != other.request_b:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def request_a(self):
        """Message field 'request_a'."""
        return self._request_a

    @request_a.setter
    def request_a(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'request_a' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'request_a' field must be an unsigned integer in [0, 255]"
        self._request_a = value

    @builtins.property
    def request_b(self):
        """Message field 'request_b'."""
        return self._request_b

    @request_b.setter
    def request_b(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'request_b' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'request_b' field must be an unsigned integer in [0, 18446744073709551615]"
        self._request_b = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_TestV2_Response(type):
    """Metaclass of message 'TestV2_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('translation_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'translation_node.srv.TestV2_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__test_v2__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__test_v2__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__test_v2__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__test_v2__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__test_v2__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TestV2_Response(metaclass=Metaclass_TestV2_Response):
    """Message class 'TestV2_Response'."""

    __slots__ = [
        '_response_a',
        '_response_b',
    ]

    _fields_and_field_types = {
        'response_a': 'uint16',
        'response_b': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.response_a = kwargs.get('response_a', int())
        self.response_b = kwargs.get('response_b', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.response_a != other.response_a:
            return False
        if self.response_b != other.response_b:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def response_a(self):
        """Message field 'response_a'."""
        return self._response_a

    @response_a.setter
    def response_a(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'response_a' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'response_a' field must be an unsigned integer in [0, 65535]"
        self._response_a = value

    @builtins.property
    def response_b(self):
        """Message field 'response_b'."""
        return self._response_b

    @response_b.setter
    def response_b(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'response_b' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'response_b' field must be an unsigned integer in [0, 18446744073709551615]"
        self._response_b = value


class Metaclass_TestV2(type):
    """Metaclass of service 'TestV2'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('translation_node')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'translation_node.srv.TestV2')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__test_v2

            from translation_node.srv import _test_v2
            if _test_v2.Metaclass_TestV2_Request._TYPE_SUPPORT is None:
                _test_v2.Metaclass_TestV2_Request.__import_type_support__()
            if _test_v2.Metaclass_TestV2_Response._TYPE_SUPPORT is None:
                _test_v2.Metaclass_TestV2_Response.__import_type_support__()


class TestV2(metaclass=Metaclass_TestV2):
    from translation_node.srv._test_v2 import TestV2_Request as Request
    from translation_node.srv._test_v2 import TestV2_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
