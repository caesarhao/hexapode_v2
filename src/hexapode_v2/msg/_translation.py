"""autogenerated by genpy from hexapode_v2/translation.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class translation(genpy.Message):
  _md5sum = "c82a53216295fbbeb0bc9b9c9bde003f"
  _type = "hexapode_v2/translation"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 dx
float64 dy
float64 da
float64 dh

"""
  __slots__ = ['dx','dy','da','dh']
  _slot_types = ['float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       dx,dy,da,dh

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(translation, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.dx is None:
        self.dx = 0.
      if self.dy is None:
        self.dy = 0.
      if self.da is None:
        self.da = 0.
      if self.dh is None:
        self.dh = 0.
    else:
      self.dx = 0.
      self.dy = 0.
      self.da = 0.
      self.dh = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_4d.pack(_x.dx, _x.dy, _x.da, _x.dh))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.dx, _x.dy, _x.da, _x.dh,) = _struct_4d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4d.pack(_x.dx, _x.dy, _x.da, _x.dh))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.dx, _x.dy, _x.da, _x.dh,) = _struct_4d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4d = struct.Struct("<4d")
