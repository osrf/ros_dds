import genmsg.msgs

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x

MSG_TYPE_TO_IDL = {
    'byte': 'octet',
    'char': 'char',
    'bool': 'boolean',
    'uint8': 'unsigned short',  # TODO reconsider mapping
    'int8': 'short',  # TODO reconsider mapping
    'uint16': 'unsigned short',
    'int16': 'short',
    'uint32': 'unsigned long',
    'int32': 'long',
    'uint64': 'unsigned long long',
    'int64': 'long long',
    'float32': 'float',
    'float64': 'double',
    'string': 'string',
    'time': 'DDS::Time',  # TODO reconsider mapping
    'duration': 'DDS::Duration'  # TODO reconsider mapping
}

#used
def msg_type_to_idl(type):
    """
    Converts a message type (e.g. uint32, std_msgs/String, etc.) into the C++ declaration
    for that type (e.g. uint32_t, std_msgs::String_<ContainerAllocator>)

    @param type: The message type
    @type type: str
    @return: The C++ declaration
    @rtype: str
    """
    (base_type, is_array, array_len) = genmsg.msgs.parse_type(type)
    idl_type = None
    if (genmsg.msgs.is_builtin(base_type)):
        idl_type = MSG_TYPE_TO_IDL[base_type]
    elif (len(base_type.split('/')) == 1):
        if (genmsg.msgs.is_header_type(base_type)):
            idl_type = 'std_msgs::dds_impl::Header_'
        else:
            idl_type = '%s' % base_type
    else:
        pkg = base_type.split('/')[0]
        msg = base_type.split('/')[1]
        idl_type = '%s::dds_impl::%s_' % (pkg, msg)

    if (is_array):
        if (array_len is None):
            return ['', '', 'sequence<%s>' % idl_type]
        else:
            typename = '%s_array_%s' % (idl_type.replace(' ', '_'), array_len)
            return ['typedef %s' % idl_type, '%s[%s];' % (typename, array_len), '%s' % typename]
    else:
        return ['', '', idl_type]

def _escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s

def escape_message_definition(definition):
    lines = definition.splitlines()
    if not lines:
        lines.append('')
    s = StringIO()
    for line in lines:
        line = _escape_string(line)
        s.write('%s\\n\\\n'%(line))
        
    val = s.getvalue()
    s.close()
    return val

#used2
def idl_message_declarations(name_prefix, msg):
    """
    Returns the different possible C++ declarations for a message given the message itself.

    @param name_prefix: The C++ prefix to be prepended to the name, e.g. "std_msgs::"
    @type name_prefix: str
    @param msg: The message type
    @type msg: str
    @return: A tuple of 3 different names.  idl_message_decelarations("std_msgs::", "String") returns the tuple
        ("std_msgs::String_", "std_msgs::String_<ContainerAllocator>", "std_msgs::String")
    @rtype: str
    """
    pkg, basetype = genmsg.names.package_resource_name(msg)
    idl_name = ' ::%s%s'%(name_prefix, msg)
    if (pkg):
        idl_name = ' ::%s::%s'%(pkg, basetype)
    return ('%s_'%(idl_name), '%s_<ContainerAllocator> '%(idl_name), '%s'%(idl_name))

#todo
def is_fixed_length(spec, msg_context, includepath):
    """
    Returns whether or not the message is fixed-length

    @param spec: The message spec
    @type spec: genmsg.msgs.MsgSpec
    @param package: The package of the
    @type package: str
    """
    types = []
    for field in spec.parsed_fields():
        if (field.is_array and field.array_len is None):
            return False

        if (field.base_type == 'string'):
            return False

        if (not field.is_builtin):
            types.append(field.base_type)

    types = set(types)
    for t in types:
        t = genmsg.msgs.resolve_type(t, spec.package)
        assert isinstance(includepath, dict)
        new_spec = genmsg.msg_loader.load_msg_by_type(msg_context, t, includepath)
        if (not is_fixed_length(new_spec, msg_context, includepath)):
            return False

    return True

#used2
def default_value(type):
    """
    Returns the value to initialize a message member with.  0 for integer types, 0.0 for floating point, false for bool,
    empty string for everything else

    @param type: The type
    @type type: str
    """
    if type in ['byte', 'int8', 'int16', 'int32', 'int64',
                'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif type in ['float32', 'float64']:
        return '0.0'
    elif type == 'bool':
        return 'false'

    return ""
#used2
def takes_allocator(type):
    """
    Returns whether or not a type can take an allocator in its constructor.  False for all builtin types except string.
    True for all others.

    @param type: The type
    @type: str
    """
    return not type in ['byte', 'int8', 'int16', 'int32', 'int64',
                        'char', 'uint8', 'uint16', 'uint32', 'uint64',
                        'float32', 'float64', 'bool', 'time', 'duration']

def escape_string(str):
    str = str.replace('\\', '\\\\')
    str = str.replace('"', '\\"')
    return str

#used
def generate_fixed_length_assigns(spec, container_gets_allocator, idl_name_prefix):
    """
    Initialize any fixed-length arrays

    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: genmsg.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    @param idl_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type idl_name_prefix: str
    """
    # Assign all fixed-length arrays their default values
    for field in spec.parsed_fields():
        if (not field.is_array or field.array_len is None):
            continue

        val = default_value(field.base_type)
        if (container_gets_allocator and takes_allocator(field.base_type)):
            # String is a special case, as it is the only builtin type that takes an allocator
            if (field.base_type == "string"):
                string_idl = msg_type_to_idl("string")
                yield '    %s.assign(%s(_alloc));\n'%(field.name, string_idl)
            else:
                (idl_msg_unqualified, idl_msg_with_alloc, _) = idl_message_declarations(idl_name_prefix, field.base_type)
                yield '    %s.assign(%s(_alloc));\n'%(field.name, idl_msg_with_alloc)
        elif (len(val) > 0):
            yield '    %s.assign(%s);\n'%(field.name, val)

#used
def generate_initializer_list(spec, container_gets_allocator):
    """
    Writes the initializer list for a constructor

    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: genmsg.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    """

    op = ':'
    for field in spec.parsed_fields():
        val = default_value(field.base_type)
        use_alloc = takes_allocator(field.base_type)
        if (field.is_array):
            if (field.array_len is None and container_gets_allocator):
                yield '  %s %s(_alloc)'%(op, field.name)
            else:
                yield '  %s %s()'%(op, field.name)
        else:
            if (container_gets_allocator and use_alloc):
                yield '  %s %s(_alloc)'%(op, field.name)
            else:
                yield '  %s %s(%s)'%(op, field.name, val)
        op = ','
