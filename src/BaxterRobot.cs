// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using RobotRaconteur;
using com.robotraconteur.robotics.robot;
using System.IO;
using System.Linq;
using ros_csharp_interop;
using com.robotraconteur.robotics.joints;
using ros_csharp_interop.rosmsg.gen.baxter_core_msgs;
using ros_csharp_interop.rosmsg.gen.std_msgs;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using com.robotraconteur.geometry;
using com.robotraconteur.action;
using com.robotraconteur.robotics.trajectory;
using RobotRaconteur.Companion.Robot;

namespace BaxterRobotRaconteurDriver
{

    public enum BaxterRobotArmSelection
    {
        both = 0,
        left = 1,
        right = 2
    }

    public class BaxterRobot : AbstractRobot, IDisposable
    {
        protected ROSNode _ros_node;
        protected string _ros_ns_prefix;
                
        protected Subscriber<ros_csharp_interop.rosmsg.gen.sensor_msgs.JointState> _joint_states_sub;
        protected Subscriber<AssemblyState> _robot_state_sub;
        protected Subscriber<EndpointState> _endpoint_state_sub;
        protected Publisher<JointCommand> _joint_command_pub_right;
        protected Publisher<JointCommand> _joint_command_pub_left;
        protected Publisher<Empty> _set_super_reset_pub;
        protected Publisher<Empty> _set_super_stop_pub;
        protected Publisher<Bool> _set_super_enable_pub;
        
        protected Publisher<DigitalOutputCommand> _digital_io_pub;
        protected List<Subscriber<DigitalIOState>> _digital_io_sub = new List<Subscriber<DigitalIOState>>();
        protected Dictionary<string, bool> _digital_io_states = new Dictionary<string, bool>();

        protected string[] _digital_io_names = {  };

        protected string[] _joint_names_right;
        protected string[] _joint_names_left;

        BaxterRobotArmSelection _arm_selection;


        public BaxterRobot(com.robotraconteur.robotics.robot.RobotInfo robot_info, BaxterRobotArmSelection arm_selection, string ros_ns_prefix = "") : base(robot_info, 14)
        {
            this._ros_ns_prefix = "";
            if (robot_info.joint_info == null)
            {
                throw new ArgumentException("Robot joint info must be provided");
            }

            _joint_names_right = new string[] { "right_s0", "right_s1", "right_e0", "right_e1",
                "right_w0", "right_w1", "right_w2" };

            _joint_names_left = new string[] { "left_s0", "left_s1", "left_e0", "left_e1",
                "left_w0", "left_w1", "left_w2"};

            switch (arm_selection)
            {
                case BaxterRobotArmSelection.both:
                    {
                        string[] _both_joint_names = new string[] { "left_s0", "left_s1", "left_e0", "left_e1",
                            "left_w0", "left_w1", "left_w2", "right_s0", "right_s1", "right_e0", "right_e1",
                            "right_w0", "right_w1", "right_w2" };

                        if (!Enumerable.SequenceEqual(_joint_names,_both_joint_names))
                        {
                            throw new ArgumentException("Invalid joint names specified for both arms");
                        }

                        _arm_selection = BaxterRobotArmSelection.both;
                        break;
                    }

                case BaxterRobotArmSelection.left:
                    {
                        if (!Enumerable.SequenceEqual(_joint_names, _joint_names_left))
                        {
                            throw new ArgumentException("Invalid joint names specified for left arm");
                        }
                        _arm_selection = BaxterRobotArmSelection.left;
                        break;
                    }
                case BaxterRobotArmSelection.right:
                    {
                        if (!Enumerable.SequenceEqual(_joint_names, _joint_names_right))
                        {
                            throw new ArgumentException("Invalid joint names specified for right arm");
                        }
                        _arm_selection = BaxterRobotArmSelection.right;
                        break;
                    }
                default:
                    throw new ArgumentException("Invalid arm selection");
            }            
        }

        

        public override void _start_robot()
        {
            //TODO: remove this


            _ros_node = new ROSNode();
            _joint_states_sub = _ros_node.subscribe<ros_csharp_interop.rosmsg.gen.sensor_msgs.JointState>(_ros_ns_prefix + "robot/joint_states", 1, _joint_state_cb); ;
            _robot_state_sub = _ros_node.subscribe<AssemblyState>(_ros_ns_prefix + "robot/state", 1, _robot_state_cb);
            if (_arm_selection != BaxterRobotArmSelection.right)
            {
                _endpoint_state_sub = _ros_node.subscribe<EndpointState>(_ros_ns_prefix + "robot/limb/left/endpoint_state", 1, x => _endpoint_state_cb(x, 0));
                _joint_command_pub_left = _ros_node.advertise<JointCommand>(_ros_ns_prefix + "robot/limb/left/joint_command", 1, false);
            }
            if (_arm_selection != BaxterRobotArmSelection.left)
            {
                _endpoint_state_sub = _ros_node.subscribe<EndpointState>(_ros_ns_prefix + "robot/limb/right/endpoint_state", 1, x => _endpoint_state_cb(x, _arm_selection == BaxterRobotArmSelection.right ? 0 : 1));
                _joint_command_pub_right = _ros_node.advertise<JointCommand>(_ros_ns_prefix + "robot/limb/right/joint_command", 1, false);
            }
            _set_super_reset_pub = _ros_node.advertise<Empty>(_ros_ns_prefix + "robot/set_super_reset", 1, false);
            _set_super_stop_pub = _ros_node.advertise<Empty>(_ros_ns_prefix + "robot/set_super_stop", 1, false);
            _set_super_enable_pub = _ros_node.advertise<Bool>(_ros_ns_prefix + "robot/set_super_enable", 1, false);
            _digital_io_pub = _ros_node.advertise<DigitalOutputCommand>(_ros_ns_prefix + "robot/digital_io/command", 10, false);

            foreach (var d in _digital_io_names)
            {
                var sub = _ros_node.subscribe<DigitalIOState>(_ros_ns_prefix + $"robot/digitial_io/{d}/state", 1, msg => _digitital_io_state_cb(d, msg));
            }

            base._start_robot();

            _ros_node.start_spinner();            
        }

        

        protected internal void _robot_state_cb(AssemblyState msg)
        {
            lock(this)
            {
                _last_robot_state = _stopwatch.ElapsedMilliseconds;
                
                // TODO: check on real robot?
                _ready = msg.enabled; //msg.ready;
                _enabled = msg.enabled;
                _stopped = msg.stopped;
                _error = msg.error;
                _estop_source = msg.estop_source;

                _operational_mode = RobotOperationalMode.cobot;
            }
        }

        protected internal void _endpoint_state_cb(EndpointState msg, int kin_chain)
        {
            lock (this)
            {
                _last_endpoint_state = _stopwatch.ElapsedMilliseconds;

                /*if (!msg.valid)
                {
                    _endpoint_pose = null;
                    _endpoint_vel = null;
                    return;
                }*/

                var p = new Pose();
                p.orientation.w = msg.pose.orientation.w;
                p.orientation.x = msg.pose.orientation.x;
                p.orientation.y = msg.pose.orientation.y;
                p.orientation.z = msg.pose.orientation.z;
                p.position.x = msg.pose.position.x;
                p.position.y = msg.pose.position.y;
                p.position.z = msg.pose.position.z;

                var v = new SpatialVelocity();
                v.angular.x = msg.twist.angular.x;
                v.angular.y = msg.twist.angular.y;
                v.angular.z = msg.twist.angular.z;
                v.linear.x = msg.twist.linear.x;
                v.linear.y = msg.twist.linear.y;
                v.linear.z = msg.twist.linear.z;

                if (_endpoint_pose == null)
                {
                    if (_arm_selection == BaxterRobotArmSelection.both)
                    {
                        _endpoint_pose = new Pose[2];
                    }
                    else
                    {
                        _endpoint_pose = new Pose[1];
                    }
                    _endpoint_pose[kin_chain] = p;
                }

                if (_endpoint_vel == null)
                {
                    _endpoint_vel = new SpatialVelocity[2];
                    _endpoint_vel[kin_chain] = v;
                }
            }
        }

        public override void Dispose()
        {
            base.Dispose();
            _joint_states_sub?.Dispose();
            _robot_state_sub?.Dispose();
            _endpoint_state_sub?.Dispose();
            _joint_command_pub_left?.Dispose();
            _joint_command_pub_right?.Dispose();
            _set_super_reset_pub?.Dispose();
            _set_super_stop_pub?.Dispose();
            _set_super_enable_pub?.Dispose();
            _digital_io_pub?.Dispose();
            foreach (var d in _digital_io_sub)
            {
                d?.Dispose();
            }

            _ros_node?.Dispose();
        }

        protected internal void _joint_state_cb(ros_csharp_interop.rosmsg.gen.sensor_msgs.JointState joint_states)
        {
            if (joint_states.name.Length < _joint_count)
            {
                return;
            }

            var joint_ind = new int[_joint_count];

            int last_ind = -1;
            for (int i=0; i<_joint_count; i++)
            {
                int last_ind_1 = last_ind + 1;
                if (last_ind_1 < joint_states.name.Length && joint_states.name[last_ind_1] == _joint_names[i])
                {
                    joint_ind[i] = last_ind_1;
                    last_ind = last_ind_1;
                    continue;
                }
                else
                {
                    bool joint_ind_found = false;
                    for (int j=0; j<joint_states.name.Length; j++)
                    {
                        if (joint_states.name[j] == _joint_names[i])
                        {
                            joint_ind_found = true;
                            joint_ind[i] = j;
                            last_ind = j;
                            break;
                        }
                    }

                    if (!joint_ind_found)
                    {
                        // We didn't find the joint name...
                        return;
                    }
                }
            }

            lock (this)
            {
                _last_joint_state = _stopwatch.ElapsedMilliseconds;

                if (joint_states.position.Length == joint_states.name.Length)
                {
                    if (_joint_position == null || _joint_position.Length != _joint_count) _joint_position = new double[_joint_count];
                    for (int i = 0; i < _joint_count; i++) _joint_position[i] = joint_states.position[joint_ind[i]];
                }
                else
                {
                    _joint_position = null;
                }

                if (joint_states.velocity.Length == joint_states.name.Length)
                {
                    if (_joint_velocity == null || _joint_velocity.Length != _joint_count) _joint_velocity = new double[_joint_count];
                    for (int i = 0; i < _joint_count; i++) _joint_velocity[i] = joint_states.velocity[joint_ind[i]];
                }
                else
                {
                    _joint_velocity = null;
                }

                if (joint_states.effort.Length == joint_states.name.Length)
                {
                    if (_joint_effort == null || _joint_effort.Length != _joint_count) _joint_effort = new double[_joint_count];
                    for (int i = 0; i < _joint_count; i++) _joint_effort[i] = joint_states.effort[joint_ind[i]];
                }
                else
                {
                    _joint_effort = null;
                }
            }                    
        }

        protected void _digitital_io_state_cb(string name, DigitalIOState state)
        {
            lock(this)
            {
                _digital_io_states[name] = state.state == 1;
            }
        }

        
        protected override Task _send_disable()
        {
            var msg = new Bool();
            msg.data = false;
            _set_super_enable_pub.publish(msg);
            return Task.FromResult(0);
        }

       
        protected override Task _send_enable()
        {
            var msg = new Bool();
            msg.data = true;
            _set_super_enable_pub.publish(msg);

            return Task.FromResult(0);
        }

        protected override Task _send_reset_errors()
        {
            var msg = new Empty();            
            _set_super_reset_pub.publish(msg);

            return Task.FromResult(0);
        }


        protected override void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd)
        {
            if (joint_pos_cmd != null)
            {
                switch (_arm_selection)
                {
                    case BaxterRobotArmSelection.left:
                        {
                            var msg_left = new JointCommand();
                            msg_left.mode = 1;
                            msg_left.names = _joint_names_left;
                            msg_left.command = joint_pos_cmd;
                            _joint_command_pub_left.publish(msg_left);
                            return;
                        }
                    case BaxterRobotArmSelection.right:
                        {
                            var msg_right = new JointCommand();
                            msg_right.mode = 1;
                            msg_right.names = _joint_names_right;
                            msg_right.command = joint_pos_cmd;
                            _joint_command_pub_right.publish(msg_right);
                            return;
                        }
                    case BaxterRobotArmSelection.both:
                        {

                            var msg_left = new JointCommand();
                            msg_left.mode = 1;
                            msg_left.names = _joint_names_left;
                            var left_pos_cmd = new double[7];
                            for (int i = 0; i < 7; i++) left_pos_cmd[i] = joint_pos_cmd[i];
                            msg_left.command = left_pos_cmd;
                            _joint_command_pub_left.publish(msg_left);

                            var msg_right = new JointCommand();
                            msg_right.mode = 1;
                            msg_right.names = _joint_names_right;
                            var right_pos_cmd = new double[7];
                            for (int i = 0; i < 7; i++) right_pos_cmd[i] = joint_pos_cmd[i + 7];
                            msg_right.command = right_pos_cmd;
                            _joint_command_pub_right.publish(msg_right);
                            return;
                        }
                    default:
                        break;
                }
            
            }

            if (joint_vel_cmd != null)
            {
                switch (_arm_selection)
                {
                    case BaxterRobotArmSelection.left:
                        {
                            var msg_left = new JointCommand();
                            msg_left.mode = 2;
                            msg_left.names = _joint_names_left;                            
                            msg_left.command = joint_vel_cmd;
                            _joint_command_pub_left.publish(msg_left);
                            return;
                        }
                    case BaxterRobotArmSelection.right:
                        {
                            var msg_right = new JointCommand();
                            msg_right.mode = 2;
                            msg_right.names = _joint_names_right;
                            msg_right.command = joint_vel_cmd;
                            _joint_command_pub_right.publish(msg_right);
                            return;
                        }
                    case BaxterRobotArmSelection.both:
                        {
                            var msg_left = new JointCommand();
                            msg_left.mode = 2;
                            msg_left.names = _joint_names_left;
                            var left_vel_cmd = new double[7];
                            for (int i = 0; i < 7; i++) left_vel_cmd[i] = joint_vel_cmd[i];
                            msg_left.command = left_vel_cmd;
                            _joint_command_pub_left.publish(msg_left);

                            var msg_right = new JointCommand();
                            msg_right.mode = 2;
                            msg_right.names = _joint_names_right;
                            var right_vel_cmd = new double[7];
                            for (int i = 0; i < 7; i++) right_vel_cmd[i] = joint_vel_cmd[i + 7];
                            msg_right.command = right_vel_cmd;
                            _joint_command_pub_right.publish(msg_right);
                            return;
                        }
                }
            }
        }
        
        
        public override Task<double[]> async_getf_signal(string signal_name, int timeout = -1)
        {
            lock (this)
            {
                if (_digital_io_states.TryGetValue(signal_name, out var state))
                {
                    if (state)
                    {
                        return Task.FromResult(new double[] { 1.0 });
                    }
                    else
                    {
                        return Task.FromResult(new double[] { 0.0 });
                    }
                }
            }

            if (_digital_io_names.Contains(signal_name))
            {
                throw new ValueNotSetException("Signal value not read");
            }

            throw new ArgumentException("Invalid signal name");            
        }

        public override Task async_setf_signal(string signal_name, double[] value_, int timeout = -1)
        {
            if (_digital_io_names.Contains(signal_name))
            {
                if (value_.Length != 1)
                {
                    throw new ArgumentException("Expected single element array for binary signal");
                }

                if (value_[0] != 0 && value_[0] != 1)
                {
                    throw new ArgumentException("Expected 0 or 1 for binary signal");
                }

                var msg = new DigitalOutputCommand();
                msg.name = signal_name;
                msg.value = value_[0] != 1.0;
                _digital_io_pub.publish(msg);
                return Task.FromResult(0);
            }

            throw new ArgumentException("Unknown signal_name");
        }
    }
}
