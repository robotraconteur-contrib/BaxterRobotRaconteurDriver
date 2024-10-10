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
using System.Diagnostics;
using com.robotraconteur.geometry;
using com.robotraconteur.identifier;
using com.robotraconteur.robotics.robot;
using com.robotraconteur.robotics.tool;
using Mono.Options;
using RobotRaconteur;
using RobotRaconteur.Companion.InfoParser;
using RobotRaconteur.Companion.Util;

namespace BaxterRobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)
        {

            var now = RobotRaconteurNode.s.NowUTC;

            bool shouldShowHelp = false;
            string robot_info_file = null;
            bool left_arm = false;
            bool right_arm = false;
            bool left_electric_gripper = false;
            bool right_electric_gripper = false;
            string left_gripper_info_file = null;
            string right_gripper_info_file = null;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "left-arm",  n => left_arm = n!=null },
                { "right-arm",  n => right_arm = n!=null },
                { "left-electric-gripper", n=>left_electric_gripper = n!=null },
                { "right-electric-gripper", n=>right_electric_gripper = n!=null },
                { "left-gripper-info-file=", n => left_gripper_info_file = n },
                { "right-gripper-info-file=", n => right_gripper_info_file = n },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null }
            };

            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("BaxterRobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `BaxterRobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: BaxterRobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                return 0;
            }

            if (robot_info_file == null)
            {
                Console.WriteLine("error: robot-info-file must be specified");
                return 1;
            }

            string nodename = "baxter_robot";

            BaxterRobotArmSelection arm_selection;
            if ((!left_arm && !right_arm) || right_arm && left_arm)
            {
                arm_selection = BaxterRobotArmSelection.both;
            }
            else if (left_arm)
            {
                arm_selection = BaxterRobotArmSelection.left;
                nodename = "baxter_robot_left_arm";
            }
            else if (right_arm)
            {
                arm_selection = BaxterRobotArmSelection.right;
                nodename = "baxter_robot_right_arm";
            }
            else
            {
                throw new ArgumentException("Invalid arm selection");
            }

            ushort port = 58660;
            if (arm_selection == BaxterRobotArmSelection.right)
            {
                port = 58661;
            }

            Tuple<RobotInfo, LocalIdentifierLocks> robot_info = null;
            Tuple<ToolInfo, LocalIdentifierLocks> left_tool_info = null;
            Tuple<ToolInfo, LocalIdentifierLocks> right_tool_info = null;

            BaxterRobot robot = null;
            BaxterElectricGripper left_gripper = null;
            BaxterElectricGripper right_gripper = null;

            try
            {
                robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);
                if (left_electric_gripper)
                {
                    left_tool_info = ToolInfoParser.LoadToolInfoYamlWithIdentifierLocks(left_gripper_info_file);
                    left_tool_info.Item1.device_info.parent_device = robot_info.Item1.device_info.device;
                    left_tool_info.Item1.device_info.device_origin_pose = new NamedPose
                    {
                        parent_frame =  new Identifier { name = "left_hand" },
                        pose = new Pose { orientation = new Quaternion { w = 1 } }
                    };
                }
                if (right_electric_gripper)
                {
                    right_tool_info = ToolInfoParser.LoadToolInfoYamlWithIdentifierLocks(right_gripper_info_file);
                    right_tool_info.Item1.device_info.parent_device = robot_info.Item1.device_info.device;
                    right_tool_info.Item1.device_info.device_origin_pose = new NamedPose
                    {
                        parent_frame = new Identifier { name = "right_hand" },
                        pose = new Pose { orientation = new Quaternion { w = 1 } }
                    };
                }

                ros_csharp_interop.ros_csharp_interop.init_ros(args, "baxter_robotraconteur_driver", true);

                robot = new BaxterRobot(robot_info.Item1, arm_selection, "");
                robot._start_robot();

                if (left_tool_info != null)
                {
                    left_gripper = new BaxterElectricGripper(left_tool_info.Item1, "left_gripper", "");
                    left_gripper._start_tool();
                }

                if (right_tool_info != null)
                {
                    right_gripper = new BaxterElectricGripper(right_tool_info.Item1, "right_gripper", "");
                    right_gripper._start_tool();
                }

                using (var node_setup = new ServerNodeSetup(nodename, port, args))
                {
                    RobotRaconteurNode.s.RegisterService("robot", "com.robotraconteur.robotics.robot", robot);
                    if (left_gripper != null)
                    {
                        RobotRaconteurNode.s.RegisterService("left_gripper", "com.robotraconteur.robotics.tool", left_gripper);
                    }
                    if (right_gripper != null)
                    {
                        RobotRaconteurNode.s.RegisterService("right_gripper", "com.robotraconteur.robotics.tool", right_gripper);
                    }

                    Console.WriteLine("Press enter to exit");
                    Console.ReadKey();
                }
                
            }
            finally
            {
                robot_info?.Item2?.Dispose();
                left_tool_info?.Item2?.Dispose();
                right_tool_info?.Item2?.Dispose();
                robot?.Dispose();
                left_gripper?.Dispose();
                right_gripper?.Dispose();
            }

            return 0;

        }
    }
}
