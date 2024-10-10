// Automatically generated, do not edit!

using System;
using System.IO;

namespace ros_csharp_interop.rosmsg.gen
{
namespace std_msgs
{
[ROSMsgInfo("std_msgs/Header","2176decaecbce78abc3b96ef049fabed","# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n")]
public class Header : ROSMsg
{
public string _type => "std_msgs/Header";
public string _md5sum => "2176decaecbce78abc3b96ef049fabed";
public string _full_text => "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
public uint seq = default;
public ROSTime stamp = default;
public string frame_id = default;
public static Header ROSRead(BinaryReader reader)
{
var o = new Header();
o.seq = rosmsg_builtin_util.read_uint(reader);
o.stamp = rosmsg_builtin_util.read_ROSTime(reader);
o.frame_id = rosmsg_builtin_util.read_string(reader);
return o;
}
public static Header[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Header[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Header msg)
{
rosmsg_builtin_util.write_uint(writer, msg.seq);
rosmsg_builtin_util.write_ROSTime(writer, msg.stamp);
rosmsg_builtin_util.write_string(writer, msg.frame_id);
}
public static void ROSWriteArray(BinaryWriter writer, Header[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace sensor_msgs
{
[ROSMsgInfo("sensor_msgs/JointState","3066dcd76a6cfaef579bd0f34173e9fd","# This is a message that holds data to describe the state of a set of torque controlled joints. \n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and \n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n")]
public class JointState : ROSMsg
{
public string _type => "sensor_msgs/JointState";
public string _md5sum => "3066dcd76a6cfaef579bd0f34173e9fd";
public string _full_text => "# This is a message that holds data to describe the state of a set of torque controlled joints. \n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and \n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n";
public std_msgs.Header header = default;
public string[] name = new string[0];
public double[] position = new double[0];
public double[] velocity = new double[0];
public double[] effort = new double[0];
public static JointState ROSRead(BinaryReader reader)
{
var o = new JointState();
o.header = std_msgs.Header.ROSRead(reader);
o.name = rosmsg_builtin_util.read_string_array(reader, -1);
o.position = rosmsg_builtin_util.read_double_array(reader, -1);
o.velocity = rosmsg_builtin_util.read_double_array(reader, -1);
o.effort = rosmsg_builtin_util.read_double_array(reader, -1);
return o;
}
public static JointState[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new JointState[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, JointState msg)
{
std_msgs.Header.ROSWrite(writer, msg.header);
rosmsg_builtin_util.write_string_array(writer, msg.name, -1);
rosmsg_builtin_util.write_double_array(writer, msg.position, -1);
rosmsg_builtin_util.write_double_array(writer, msg.velocity, -1);
rosmsg_builtin_util.write_double_array(writer, msg.effort, -1);
}
public static void ROSWriteArray(BinaryWriter writer, JointState[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/AssemblyState","356d9dd237ce73b2667da9235f541933","bool ready               # true if enabled and ready to operate, e.g., not homing\nbool enabled             # true if enabled\nbool stopped             # true if stopped -- e-stop asserted\nbool error               # true if a component of the assembly has an error\n#\n# The following are specific to the robot top-level assembly:\nuint8  estop_button      # One of the following:\n  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n  uint8   ESTOP_BUTTON_PRESSED   = 1\n  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n#\nuint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n  uint8  ESTOP_SOURCE_BRAIN     = 4   # MotorController asserted e-stop in response to a lapse of the brain heartbeat\n")]
public class AssemblyState : ROSMsg
{
public string _type => "baxter_core_msgs/AssemblyState";
public string _md5sum => "356d9dd237ce73b2667da9235f541933";
public string _full_text => "bool ready               # true if enabled and ready to operate, e.g., not homing\nbool enabled             # true if enabled\nbool stopped             # true if stopped -- e-stop asserted\nbool error               # true if a component of the assembly has an error\n#\n# The following are specific to the robot top-level assembly:\nuint8  estop_button      # One of the following:\n  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n  uint8   ESTOP_BUTTON_PRESSED   = 1\n  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n#\nuint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n  uint8  ESTOP_SOURCE_BRAIN     = 4   # MotorController asserted e-stop in response to a lapse of the brain heartbeat\n";
public bool ready = default;
public bool enabled = default;
public bool stopped = default;
public bool error = default;
public byte estop_button = default;
public byte estop_source = default;
public static AssemblyState ROSRead(BinaryReader reader)
{
var o = new AssemblyState();
o.ready = rosmsg_builtin_util.read_bool(reader);
o.enabled = rosmsg_builtin_util.read_bool(reader);
o.stopped = rosmsg_builtin_util.read_bool(reader);
o.error = rosmsg_builtin_util.read_bool(reader);
o.estop_button = rosmsg_builtin_util.read_byte(reader);
o.estop_source = rosmsg_builtin_util.read_byte(reader);
return o;
}
public static AssemblyState[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new AssemblyState[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, AssemblyState msg)
{
rosmsg_builtin_util.write_bool(writer, msg.ready);
rosmsg_builtin_util.write_bool(writer, msg.enabled);
rosmsg_builtin_util.write_bool(writer, msg.stopped);
rosmsg_builtin_util.write_bool(writer, msg.error);
rosmsg_builtin_util.write_byte(writer, msg.estop_button);
rosmsg_builtin_util.write_byte(writer, msg.estop_source);
}
public static void ROSWriteArray(BinaryWriter writer, AssemblyState[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/JointCommand","19bfec8434dd568ab3c633d187c36f2e","int32 mode\nfloat64[] command\nstring[]  names\n\nint32 POSITION_MODE=1\nint32 VELOCITY_MODE=2\nint32 TORQUE_MODE=3\nint32 RAW_POSITION_MODE=4")]
public class JointCommand : ROSMsg
{
public string _type => "baxter_core_msgs/JointCommand";
public string _md5sum => "19bfec8434dd568ab3c633d187c36f2e";
public string _full_text => "int32 mode\nfloat64[] command\nstring[]  names\n\nint32 POSITION_MODE=1\nint32 VELOCITY_MODE=2\nint32 TORQUE_MODE=3\nint32 RAW_POSITION_MODE=4";
public int mode = default;
public double[] command = new double[0];
public string[] names = new string[0];
public static JointCommand ROSRead(BinaryReader reader)
{
var o = new JointCommand();
o.mode = rosmsg_builtin_util.read_int(reader);
o.command = rosmsg_builtin_util.read_double_array(reader, -1);
o.names = rosmsg_builtin_util.read_string_array(reader, -1);
return o;
}
public static JointCommand[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new JointCommand[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, JointCommand msg)
{
rosmsg_builtin_util.write_int(writer, msg.mode);
rosmsg_builtin_util.write_double_array(writer, msg.command, -1);
rosmsg_builtin_util.write_string_array(writer, msg.names, -1);
}
public static void ROSWriteArray(BinaryWriter writer, JointCommand[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace std_msgs
{
[ROSMsgInfo("std_msgs/Empty","d41d8cd98f00b204e9800998ecf8427e","")]
public class Empty : ROSMsg
{
public string _type => "std_msgs/Empty";
public string _md5sum => "d41d8cd98f00b204e9800998ecf8427e";
public string _full_text => "";
public static Empty ROSRead(BinaryReader reader)
{
var o = new Empty();
return o;
}
public static Empty[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Empty[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Empty msg)
{
}
public static void ROSWriteArray(BinaryWriter writer, Empty[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace std_msgs
{
[ROSMsgInfo("std_msgs/Bool","8b94c1b53db61fb6aed406028ad6332a","bool data")]
public class Bool : ROSMsg
{
public string _type => "std_msgs/Bool";
public string _md5sum => "8b94c1b53db61fb6aed406028ad6332a";
public string _full_text => "bool data";
public bool data = default;
public static Bool ROSRead(BinaryReader reader)
{
var o = new Bool();
o.data = rosmsg_builtin_util.read_bool(reader);
return o;
}
public static Bool[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Bool[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Bool msg)
{
rosmsg_builtin_util.write_bool(writer, msg.data);
}
public static void ROSWriteArray(BinaryWriter writer, Bool[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/EndpointState","44bea01d596ff699fa1447bec34167ac","Header header\ngeometry_msgs/Pose   pose\ngeometry_msgs/Twist  twist\ngeometry_msgs/Wrench wrench\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n")]
public class EndpointState : ROSMsg
{
public string _type => "baxter_core_msgs/EndpointState";
public string _md5sum => "44bea01d596ff699fa1447bec34167ac";
public string _full_text => "Header header\ngeometry_msgs/Pose   pose\ngeometry_msgs/Twist  twist\ngeometry_msgs/Wrench wrench\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\nstring frame_id\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n================================================================================\nMSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z\n================================================================================\nMSG: geometry_msgs/Wrench\n# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n";
public std_msgs.Header header = default;
public geometry_msgs.Pose pose = default;
public geometry_msgs.Twist twist = default;
public geometry_msgs.Wrench wrench = default;
public static EndpointState ROSRead(BinaryReader reader)
{
var o = new EndpointState();
o.header = std_msgs.Header.ROSRead(reader);
o.pose = geometry_msgs.Pose.ROSRead(reader);
o.twist = geometry_msgs.Twist.ROSRead(reader);
o.wrench = geometry_msgs.Wrench.ROSRead(reader);
return o;
}
public static EndpointState[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new EndpointState[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, EndpointState msg)
{
std_msgs.Header.ROSWrite(writer, msg.header);
geometry_msgs.Pose.ROSWrite(writer, msg.pose);
geometry_msgs.Twist.ROSWrite(writer, msg.twist);
geometry_msgs.Wrench.ROSWrite(writer, msg.wrench);
}
public static void ROSWriteArray(BinaryWriter writer, EndpointState[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Pose","e45d45a5a1ce597b249e23fb30fc871f","# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")]
public class Pose : ROSMsg
{
public string _type => "geometry_msgs/Pose";
public string _md5sum => "e45d45a5a1ce597b249e23fb30fc871f";
public string _full_text => "# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
public geometry_msgs.Point position = default;
public geometry_msgs.Quaternion orientation = default;
public static Pose ROSRead(BinaryReader reader)
{
var o = new Pose();
o.position = geometry_msgs.Point.ROSRead(reader);
o.orientation = geometry_msgs.Quaternion.ROSRead(reader);
return o;
}
public static Pose[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Pose[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Pose msg)
{
geometry_msgs.Point.ROSWrite(writer, msg.position);
geometry_msgs.Quaternion.ROSWrite(writer, msg.orientation);
}
public static void ROSWriteArray(BinaryWriter writer, Pose[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Twist","9f195f881246fdfa2798d1d3eebca84a","# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
public class Twist : ROSMsg
{
public string _type => "geometry_msgs/Twist";
public string _md5sum => "9f195f881246fdfa2798d1d3eebca84a";
public string _full_text => "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
public geometry_msgs.Vector3 linear = default;
public geometry_msgs.Vector3 angular = default;
public static Twist ROSRead(BinaryReader reader)
{
var o = new Twist();
o.linear = geometry_msgs.Vector3.ROSRead(reader);
o.angular = geometry_msgs.Vector3.ROSRead(reader);
return o;
}
public static Twist[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Twist[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Twist msg)
{
geometry_msgs.Vector3.ROSWrite(writer, msg.linear);
geometry_msgs.Vector3.ROSWrite(writer, msg.angular);
}
public static void ROSWriteArray(BinaryWriter writer, Twist[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Wrench","4f539cf138b23283b520fd271b567936","# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
public class Wrench : ROSMsg
{
public string _type => "geometry_msgs/Wrench";
public string _md5sum => "4f539cf138b23283b520fd271b567936";
public string _full_text => "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n\n================================================================================\nMSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
public geometry_msgs.Vector3 force = default;
public geometry_msgs.Vector3 torque = default;
public static Wrench ROSRead(BinaryReader reader)
{
var o = new Wrench();
o.force = geometry_msgs.Vector3.ROSRead(reader);
o.torque = geometry_msgs.Vector3.ROSRead(reader);
return o;
}
public static Wrench[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Wrench[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Wrench msg)
{
geometry_msgs.Vector3.ROSWrite(writer, msg.force);
geometry_msgs.Vector3.ROSWrite(writer, msg.torque);
}
public static void ROSWriteArray(BinaryWriter writer, Wrench[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Point","4a842b65f413084dc2b10fb484ea7f17","# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n")]
public class Point : ROSMsg
{
public string _type => "geometry_msgs/Point";
public string _md5sum => "4a842b65f413084dc2b10fb484ea7f17";
public string _full_text => "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
public double x = default;
public double y = default;
public double z = default;
public static Point ROSRead(BinaryReader reader)
{
var o = new Point();
o.x = rosmsg_builtin_util.read_double(reader);
o.y = rosmsg_builtin_util.read_double(reader);
o.z = rosmsg_builtin_util.read_double(reader);
return o;
}
public static Point[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Point[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Point msg)
{
rosmsg_builtin_util.write_double(writer, msg.x);
rosmsg_builtin_util.write_double(writer, msg.y);
rosmsg_builtin_util.write_double(writer, msg.z);
}
public static void ROSWriteArray(BinaryWriter writer, Point[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Quaternion","a779879fadf0160734f906b8c19c7004","# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n")]
public class Quaternion : ROSMsg
{
public string _type => "geometry_msgs/Quaternion";
public string _md5sum => "a779879fadf0160734f906b8c19c7004";
public string _full_text => "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
public double x = default;
public double y = default;
public double z = default;
public double w = default;
public static Quaternion ROSRead(BinaryReader reader)
{
var o = new Quaternion();
o.x = rosmsg_builtin_util.read_double(reader);
o.y = rosmsg_builtin_util.read_double(reader);
o.z = rosmsg_builtin_util.read_double(reader);
o.w = rosmsg_builtin_util.read_double(reader);
return o;
}
public static Quaternion[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Quaternion[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Quaternion msg)
{
rosmsg_builtin_util.write_double(writer, msg.x);
rosmsg_builtin_util.write_double(writer, msg.y);
rosmsg_builtin_util.write_double(writer, msg.z);
rosmsg_builtin_util.write_double(writer, msg.w);
}
public static void ROSWriteArray(BinaryWriter writer, Quaternion[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace geometry_msgs
{
[ROSMsgInfo("geometry_msgs/Vector3","4a842b65f413084dc2b10fb484ea7f17","# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z")]
public class Vector3 : ROSMsg
{
public string _type => "geometry_msgs/Vector3";
public string _md5sum => "4a842b65f413084dc2b10fb484ea7f17";
public string _full_text => "# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
public double x = default;
public double y = default;
public double z = default;
public static Vector3 ROSRead(BinaryReader reader)
{
var o = new Vector3();
o.x = rosmsg_builtin_util.read_double(reader);
o.y = rosmsg_builtin_util.read_double(reader);
o.z = rosmsg_builtin_util.read_double(reader);
return o;
}
public static Vector3[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new Vector3[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, Vector3 msg)
{
rosmsg_builtin_util.write_double(writer, msg.x);
rosmsg_builtin_util.write_double(writer, msg.y);
rosmsg_builtin_util.write_double(writer, msg.z);
}
public static void ROSWriteArray(BinaryWriter writer, Vector3[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/DigitalIOState","29d0be3859dae81a66b28f167ecec98c","int8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0")]
public class DigitalIOState : ROSMsg
{
public string _type => "baxter_core_msgs/DigitalIOState";
public string _md5sum => "29d0be3859dae81a66b28f167ecec98c";
public string _full_text => "int8 state\nbool isInputOnly\n\nint8 OFF = 0\nint8 ON  = 1\nint8 PRESSED = 1\nint8 UNPRESSED = 0";
public sbyte state = default;
public bool isInputOnly = default;
public static DigitalIOState ROSRead(BinaryReader reader)
{
var o = new DigitalIOState();
o.state = rosmsg_builtin_util.read_sbyte(reader);
o.isInputOnly = rosmsg_builtin_util.read_bool(reader);
return o;
}
public static DigitalIOState[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new DigitalIOState[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, DigitalIOState msg)
{
rosmsg_builtin_util.write_sbyte(writer, msg.state);
rosmsg_builtin_util.write_bool(writer, msg.isInputOnly);
}
public static void ROSWriteArray(BinaryWriter writer, DigitalIOState[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/EndEffectorState","ade777f069d738595bc19e246b8ec7a0","#\ntime timestamp              # time when state was updated\nuint32 id                   # EndEffectorId\n#\n# The following State fields are tristate: 0 = false; 1 = true; 2 = unknown/unsupported\n  uint8   STATE_FALSE = 0\n  uint8   STATE_TRUE = 1\n  uint8   STATE_UNKNOWN = 2\n#\nuint8   enabled             # true if enabled\nuint8   calibrated          # true if calibration has completed\nuint8   ready               # true if ready for another command\nuint8   moving              # true if moving\nuint8   gripping            # true if gripping\nuint8   missed              # true if GRIP/GOTO/SET was commanded and the gripper reaches the end of travel\nuint8   error               # true if the gripper is in an error state\nuint8   reverse             # true if the gripper is in reverse mode\n#\nfloat32 position            # position as a percentage of the max position;      0=closed - 100=open\n#\n  float32 POSITION_CLOSED = 0.0\n  float32 POSITION_OPEN   = 100.0\n#\nfloat32 force               # force as a percentage of max force;                0=none   - 100=max\n#\n  float32 FORCE_MIN = 0.0\n  float32 FORCE_MAX = 100.0\n#\nstring state                # JSON: other state information\n#\nstring command              # from the last command message\nstring command_sender\nuint32 command_sequence\n#\n")]
public class EndEffectorState : ROSMsg
{
public string _type => "baxter_core_msgs/EndEffectorState";
public string _md5sum => "ade777f069d738595bc19e246b8ec7a0";
public string _full_text => "#\ntime timestamp              # time when state was updated\nuint32 id                   # EndEffectorId\n#\n# The following State fields are tristate: 0 = false; 1 = true; 2 = unknown/unsupported\n  uint8   STATE_FALSE = 0\n  uint8   STATE_TRUE = 1\n  uint8   STATE_UNKNOWN = 2\n#\nuint8   enabled             # true if enabled\nuint8   calibrated          # true if calibration has completed\nuint8   ready               # true if ready for another command\nuint8   moving              # true if moving\nuint8   gripping            # true if gripping\nuint8   missed              # true if GRIP/GOTO/SET was commanded and the gripper reaches the end of travel\nuint8   error               # true if the gripper is in an error state\nuint8   reverse             # true if the gripper is in reverse mode\n#\nfloat32 position            # position as a percentage of the max position;      0=closed - 100=open\n#\n  float32 POSITION_CLOSED = 0.0\n  float32 POSITION_OPEN   = 100.0\n#\nfloat32 force               # force as a percentage of max force;                0=none   - 100=max\n#\n  float32 FORCE_MIN = 0.0\n  float32 FORCE_MAX = 100.0\n#\nstring state                # JSON: other state information\n#\nstring command              # from the last command message\nstring command_sender\nuint32 command_sequence\n#\n";
public ROSTime timestamp = default;
public uint id = default;
public byte enabled = default;
public byte calibrated = default;
public byte ready = default;
public byte moving = default;
public byte gripping = default;
public byte missed = default;
public byte error = default;
public byte reverse = default;
public float position = default;
public float force = default;
public string state = default;
public string command = default;
public string command_sender = default;
public uint command_sequence = default;
public static EndEffectorState ROSRead(BinaryReader reader)
{
var o = new EndEffectorState();
o.timestamp = rosmsg_builtin_util.read_ROSTime(reader);
o.id = rosmsg_builtin_util.read_uint(reader);
o.enabled = rosmsg_builtin_util.read_byte(reader);
o.calibrated = rosmsg_builtin_util.read_byte(reader);
o.ready = rosmsg_builtin_util.read_byte(reader);
o.moving = rosmsg_builtin_util.read_byte(reader);
o.gripping = rosmsg_builtin_util.read_byte(reader);
o.missed = rosmsg_builtin_util.read_byte(reader);
o.error = rosmsg_builtin_util.read_byte(reader);
o.reverse = rosmsg_builtin_util.read_byte(reader);
o.position = rosmsg_builtin_util.read_float(reader);
o.force = rosmsg_builtin_util.read_float(reader);
o.state = rosmsg_builtin_util.read_string(reader);
o.command = rosmsg_builtin_util.read_string(reader);
o.command_sender = rosmsg_builtin_util.read_string(reader);
o.command_sequence = rosmsg_builtin_util.read_uint(reader);
return o;
}
public static EndEffectorState[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new EndEffectorState[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, EndEffectorState msg)
{
rosmsg_builtin_util.write_ROSTime(writer, msg.timestamp);
rosmsg_builtin_util.write_uint(writer, msg.id);
rosmsg_builtin_util.write_byte(writer, msg.enabled);
rosmsg_builtin_util.write_byte(writer, msg.calibrated);
rosmsg_builtin_util.write_byte(writer, msg.ready);
rosmsg_builtin_util.write_byte(writer, msg.moving);
rosmsg_builtin_util.write_byte(writer, msg.gripping);
rosmsg_builtin_util.write_byte(writer, msg.missed);
rosmsg_builtin_util.write_byte(writer, msg.error);
rosmsg_builtin_util.write_byte(writer, msg.reverse);
rosmsg_builtin_util.write_float(writer, msg.position);
rosmsg_builtin_util.write_float(writer, msg.force);
rosmsg_builtin_util.write_string(writer, msg.state);
rosmsg_builtin_util.write_string(writer, msg.command);
rosmsg_builtin_util.write_string(writer, msg.command_sender);
rosmsg_builtin_util.write_uint(writer, msg.command_sequence);
}
public static void ROSWriteArray(BinaryWriter writer, EndEffectorState[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/DigitalOutputCommand","23f05028c1a699fb83e22401228c3a9e","##the name of the output\nstring name  \n##the value to set output \nbool value   \n")]
public class DigitalOutputCommand : ROSMsg
{
public string _type => "baxter_core_msgs/DigitalOutputCommand";
public string _md5sum => "23f05028c1a699fb83e22401228c3a9e";
public string _full_text => "##the name of the output\nstring name  \n##the value to set output \nbool value   \n";
public string name = default;
public bool value = default;
public static DigitalOutputCommand ROSRead(BinaryReader reader)
{
var o = new DigitalOutputCommand();
o.name = rosmsg_builtin_util.read_string(reader);
o.value = rosmsg_builtin_util.read_bool(reader);
return o;
}
public static DigitalOutputCommand[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new DigitalOutputCommand[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, DigitalOutputCommand msg)
{
rosmsg_builtin_util.write_string(writer, msg.name);
rosmsg_builtin_util.write_bool(writer, msg.value);
}
public static void ROSWriteArray(BinaryWriter writer, DigitalOutputCommand[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
namespace baxter_core_msgs
{
[ROSMsgInfo("baxter_core_msgs/EndEffectorCommand","c003234e90416f2ca02ac7837c42cbb7","## Command to be sent to an end effector\nuint32 id       # target end effector id\nstring command  # operation to perform\n# Well known commands:\nstring   CMD_NO_OP           = no_op\nstring   CMD_SET             = set\nstring   CMD_CONFIGURE       = configure\nstring   CMD_REBOOT          = reboot\nstring   CMD_RESET           = reset\nstring   CMD_CALIBRATE       = calibrate\nstring   CMD_CLEAR_CALIBRATION = clear_calibration\nstring   CMD_PREPARE_TO_GRIP = prepare_to_grip\nstring   CMD_GRIP            = grip\nstring   CMD_RELEASE         = release\nstring   CMD_GO              = go\nstring   CMD_STOP            = stop\n#\nstring args     # JSON arguments to the command\n#\nstring sender   # optional identifier, returned in state when the command is handled\nuint32 sequence # optional sequence number, return in state when the command is handled\n\n")]
public class EndEffectorCommand : ROSMsg
{
public string _type => "baxter_core_msgs/EndEffectorCommand";
public string _md5sum => "c003234e90416f2ca02ac7837c42cbb7";
public string _full_text => "## Command to be sent to an end effector\nuint32 id       # target end effector id\nstring command  # operation to perform\n# Well known commands:\nstring   CMD_NO_OP           = no_op\nstring   CMD_SET             = set\nstring   CMD_CONFIGURE       = configure\nstring   CMD_REBOOT          = reboot\nstring   CMD_RESET           = reset\nstring   CMD_CALIBRATE       = calibrate\nstring   CMD_CLEAR_CALIBRATION = clear_calibration\nstring   CMD_PREPARE_TO_GRIP = prepare_to_grip\nstring   CMD_GRIP            = grip\nstring   CMD_RELEASE         = release\nstring   CMD_GO              = go\nstring   CMD_STOP            = stop\n#\nstring args     # JSON arguments to the command\n#\nstring sender   # optional identifier, returned in state when the command is handled\nuint32 sequence # optional sequence number, return in state when the command is handled\n\n";
public uint id = default;
public string command = default;
public string args = default;
public string sender = default;
public uint sequence = default;
public static EndEffectorCommand ROSRead(BinaryReader reader)
{
var o = new EndEffectorCommand();
o.id = rosmsg_builtin_util.read_uint(reader);
o.command = rosmsg_builtin_util.read_string(reader);
o.args = rosmsg_builtin_util.read_string(reader);
o.sender = rosmsg_builtin_util.read_string(reader);
o.sequence = rosmsg_builtin_util.read_uint(reader);
return o;
}
public static EndEffectorCommand[] ROSReadArray(BinaryReader reader, int count)
{
if (count < 0) count = (int)reader.ReadUInt32();
var o = new EndEffectorCommand[count];
for (int i=0; i<count; i++) o[i] = ROSRead(reader);
return o;
}
public static void ROSWrite(BinaryWriter writer, EndEffectorCommand msg)
{
rosmsg_builtin_util.write_uint(writer, msg.id);
rosmsg_builtin_util.write_string(writer, msg.command);
rosmsg_builtin_util.write_string(writer, msg.args);
rosmsg_builtin_util.write_string(writer, msg.sender);
rosmsg_builtin_util.write_uint(writer, msg.sequence);
}
public static void ROSWriteArray(BinaryWriter writer, EndEffectorCommand[] msg, int count)
{
rosmsg_builtin_util.do_write_count(writer,msg,count);
for (int i=0; i<msg.Length; i++) ROSWrite(writer, msg[i]);
}
}
}
}
