using com.robotraconteur.robotics.tool;
using RobotRaconteur.Companion.Robot;
using ros_csharp_interop;
using ros_csharp_interop.rosmsg.gen.baxter_core_msgs;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace BaxterRobotRaconteurDriver
{
    public class BaxterElectricGripper : AbstractTool
    {
        protected ROSNode _ros_node;
        protected string _ros_ns_prefix;
        protected string _ros_tool_name;

        protected Subscriber<EndEffectorState> _gripper_state_sub;
        protected Publisher<EndEffectorCommand> _gripper_command_pub;

        protected EndEffectorState _gripper_current_state;
        protected long _last_gripper_state;
        protected double _last_command;
        protected uint _tool_id;
        protected uint _ros_cmd_seqno = 0;

        public BaxterElectricGripper(ToolInfo tool_info, string ros_tool_name, string ros_ns_prefix = "") : base(tool_info)
        {
            this._ros_ns_prefix = "";
            this._ros_tool_name = ros_tool_name;
        }


        public override void _start_tool()
        {
            _ros_node = new ROSNode();
            _gripper_state_sub = _ros_node.subscribe<EndEffectorState>(_ros_ns_prefix + "robot/end_effector/" + _ros_tool_name + "/state", 1, _gripper_state_cb); ;
            _gripper_command_pub = _ros_node.advertise<EndEffectorCommand>(_ros_ns_prefix + "robot/end_effector/" + _ros_tool_name + "/command", 1, false);

            base._start_tool();
        }

        private void _gripper_state_cb(EndEffectorState obj)
        {
            lock(this)
            {
                _tool_id = obj.id;
                _gripper_current_state = obj;
                _last_gripper_state = _stopwatch.ElapsedMilliseconds;
            }
        }

        protected override void _fill_state(long now, out ToolState rr_tool_state)
        {
            lock (this)
            {
                var o = new ToolState();
                o.seqno = _state_seqno;
                o.command = _last_command;

                if (_gripper_current_state == null || (now - _last_gripper_state > 2000))
                {
                    o.tool_state_flags = (uint)ToolStateFlags.communication_failure;
                    o.sensor = new double[0];
                }
                else
                {
                    o.position = _gripper_current_state.position;
                    o.sensor = new double[] { _gripper_current_state.force };

                    uint f = 0;
                    if (_gripper_current_state.error == 1)
                    {
                        f |= (uint)ToolStateFlags.error;
                    }
                    if (_gripper_current_state.enabled == 1)
                    {
                        f |= (uint)ToolStateFlags.enabled;
                    }
                    if (_gripper_current_state.ready == 1)
                    {
                        f |= (uint)ToolStateFlags.ready;
                    }
                    if (_gripper_current_state.calibrated == 1)
                    {
                        f |= (uint)ToolStateFlags.homed;
                    }
                    else
                    {
                        f |= (uint)ToolStateFlags.requires_homing;
                    }
                    if (_gripper_current_state.gripping == 1)
                    {
                        f |= (uint)ToolStateFlags.gripping;
                    }
                    if (_gripper_current_state.gripping == 1)
                    {
                        f |= (uint)ToolStateFlags.missed;
                    }
                    if (_gripper_current_state.moving == 1)
                    {
                        f |= (uint)ToolStateFlags.actuating;
                    }
                    if (_position < 5)
                    {
                        f |= (uint)ToolStateFlags.closed;
                    }
                    else if (_position > 95)
                    {
                        f |= (uint)ToolStateFlags.opened;
                    }
                    else
                    {
                        f |= (uint)ToolStateFlags.between;
                    }
                    o.tool_state_flags = f;
                }

                rr_tool_state = o;
                
            }
        }

        public override void home()
        {
            uint cmd_seqno;
            lock(this)
            {
                cmd_seqno = ++_ros_cmd_seqno;
            }
            var cmd1 = new EndEffectorCommand();
            cmd1.sequence = cmd_seqno;
            cmd1.command = "clear_calibration";
            cmd1.id = _tool_id;
            _gripper_command_pub.publish(cmd1);

            Thread.Sleep(1000);

            lock (this)
            {
                cmd_seqno = ++_ros_cmd_seqno;
            }
            var cmd2 = new EndEffectorCommand();
            cmd2.sequence = cmd_seqno;
            cmd2.command = "calibrate";
            cmd2.id = _tool_id;
            _gripper_command_pub.publish(cmd2);
            lock (this)
            {
                _command = 0;
            }
        }

        public override void close()
        {
            setf_command(0);
        }

        public override void open()
        {
            setf_command(100);
        }

        public override void halt()
        {
            uint cmd_seqno;
            lock (this)
            {
                cmd_seqno = ++_ros_cmd_seqno;
            }
            var cmd1 = new EndEffectorCommand();
            cmd1.sequence = cmd_seqno;
            cmd1.command = "halt";
            cmd1.id = _tool_id;
            _gripper_command_pub.publish(cmd1);
        }
        public override void setf_command(double command)
        {
            if (command < 0 || command > 100)
            {
                throw new ArgumentException("Command must be between 0 and 100");
            }

            uint cmd_seqno;
            lock (this)
            {
                cmd_seqno = ++_ros_cmd_seqno;
            }

            var cmd1 = new EndEffectorCommand();
            cmd1.sequence = cmd_seqno;
            cmd1.id = _tool_id;
            cmd1.command = "go";
            cmd1.args = "{\"position\": " + command + "}";
            _gripper_command_pub.publish(cmd1);

            lock(this)
            {
                _command = command;
            }
        }

        public override void Dispose()
        {
            base.Dispose();
            _gripper_state_sub?.Dispose();

        }

        
    }
}
