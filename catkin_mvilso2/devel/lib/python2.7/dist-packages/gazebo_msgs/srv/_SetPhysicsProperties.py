# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from gazebo_msgs/SetPhysicsPropertiesRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import gazebo_msgs.msg
import geometry_msgs.msg

class SetPhysicsPropertiesRequest(genpy.Message):
  _md5sum = "abd9f82732b52b92e9d6bb36e6a82452"
  _type = "gazebo_msgs/SetPhysicsPropertiesRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
float64 time_step
float64 max_update_rate
geometry_msgs/Vector3 gravity
gazebo_msgs/ODEPhysics ode_config

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: gazebo_msgs/ODEPhysics
bool auto_disable_bodies           # enable auto disabling of bodies, default false
uint32 sor_pgs_precon_iters        # preconditioning inner iterations when uisng projected Gauss Seidel
uint32 sor_pgs_iters               # inner iterations when uisng projected Gauss Seidel
float64 sor_pgs_w                  # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
float64 sor_pgs_rms_error_tol      # rms error tolerance before stopping inner iterations
float64 contact_surface_layer      # contact "dead-band" width
float64 contact_max_correcting_vel # contact maximum correction velocity
float64 cfm                        # global constraint force mixing
float64 erp                        # global error reduction parameter
uint32 max_contacts                # maximum contact joints between two geoms
"""
  __slots__ = ['time_step','max_update_rate','gravity','ode_config']
  _slot_types = ['float64','float64','geometry_msgs/Vector3','gazebo_msgs/ODEPhysics']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       time_step,max_update_rate,gravity,ode_config

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetPhysicsPropertiesRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.time_step is None:
        self.time_step = 0.
      if self.max_update_rate is None:
        self.max_update_rate = 0.
      if self.gravity is None:
        self.gravity = geometry_msgs.msg.Vector3()
      if self.ode_config is None:
        self.ode_config = gazebo_msgs.msg.ODEPhysics()
    else:
      self.time_step = 0.
      self.max_update_rate = 0.
      self.gravity = geometry_msgs.msg.Vector3()
      self.ode_config = gazebo_msgs.msg.ODEPhysics()

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
      buff.write(_get_struct_5dB2I6dI().pack(_x.time_step, _x.max_update_rate, _x.gravity.x, _x.gravity.y, _x.gravity.z, _x.ode_config.auto_disable_bodies, _x.ode_config.sor_pgs_precon_iters, _x.ode_config.sor_pgs_iters, _x.ode_config.sor_pgs_w, _x.ode_config.sor_pgs_rms_error_tol, _x.ode_config.contact_surface_layer, _x.ode_config.contact_max_correcting_vel, _x.ode_config.cfm, _x.ode_config.erp, _x.ode_config.max_contacts))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.gravity is None:
        self.gravity = geometry_msgs.msg.Vector3()
      if self.ode_config is None:
        self.ode_config = gazebo_msgs.msg.ODEPhysics()
      end = 0
      _x = self
      start = end
      end += 101
      (_x.time_step, _x.max_update_rate, _x.gravity.x, _x.gravity.y, _x.gravity.z, _x.ode_config.auto_disable_bodies, _x.ode_config.sor_pgs_precon_iters, _x.ode_config.sor_pgs_iters, _x.ode_config.sor_pgs_w, _x.ode_config.sor_pgs_rms_error_tol, _x.ode_config.contact_surface_layer, _x.ode_config.contact_max_correcting_vel, _x.ode_config.cfm, _x.ode_config.erp, _x.ode_config.max_contacts,) = _get_struct_5dB2I6dI().unpack(str[start:end])
      self.ode_config.auto_disable_bodies = bool(self.ode_config.auto_disable_bodies)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_5dB2I6dI().pack(_x.time_step, _x.max_update_rate, _x.gravity.x, _x.gravity.y, _x.gravity.z, _x.ode_config.auto_disable_bodies, _x.ode_config.sor_pgs_precon_iters, _x.ode_config.sor_pgs_iters, _x.ode_config.sor_pgs_w, _x.ode_config.sor_pgs_rms_error_tol, _x.ode_config.contact_surface_layer, _x.ode_config.contact_max_correcting_vel, _x.ode_config.cfm, _x.ode_config.erp, _x.ode_config.max_contacts))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.gravity is None:
        self.gravity = geometry_msgs.msg.Vector3()
      if self.ode_config is None:
        self.ode_config = gazebo_msgs.msg.ODEPhysics()
      end = 0
      _x = self
      start = end
      end += 101
      (_x.time_step, _x.max_update_rate, _x.gravity.x, _x.gravity.y, _x.gravity.z, _x.ode_config.auto_disable_bodies, _x.ode_config.sor_pgs_precon_iters, _x.ode_config.sor_pgs_iters, _x.ode_config.sor_pgs_w, _x.ode_config.sor_pgs_rms_error_tol, _x.ode_config.contact_surface_layer, _x.ode_config.contact_max_correcting_vel, _x.ode_config.cfm, _x.ode_config.erp, _x.ode_config.max_contacts,) = _get_struct_5dB2I6dI().unpack(str[start:end])
      self.ode_config.auto_disable_bodies = bool(self.ode_config.auto_disable_bodies)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5dB2I6dI = None
def _get_struct_5dB2I6dI():
    global _struct_5dB2I6dI
    if _struct_5dB2I6dI is None:
        _struct_5dB2I6dI = struct.Struct("<5dB2I6dI")
    return _struct_5dB2I6dI
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from gazebo_msgs/SetPhysicsPropertiesResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetPhysicsPropertiesResponse(genpy.Message):
  _md5sum = "2ec6f3eff0161f4257b808b12bc830c2"
  _type = "gazebo_msgs/SetPhysicsPropertiesResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool success
string status_message

"""
  __slots__ = ['success','status_message']
  _slot_types = ['bool','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success,status_message

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetPhysicsPropertiesResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
      if self.status_message is None:
        self.status_message = ''
    else:
      self.success = False
      self.status_message = ''

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
      _x = self.success
      buff.write(_get_struct_B().pack(_x))
      _x = self.status_message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _get_struct_B().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status_message = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status_message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.success
      buff.write(_get_struct_B().pack(_x))
      _x = self.status_message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _get_struct_B().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status_message = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status_message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class SetPhysicsProperties(object):
  _type          = 'gazebo_msgs/SetPhysicsProperties'
  _md5sum = '97e2057080558ce4730434b5fae75c91'
  _request_class  = SetPhysicsPropertiesRequest
  _response_class = SetPhysicsPropertiesResponse
