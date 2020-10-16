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
using Mono.Options;
using RobotRaconteur;
using RobotRaconteur.Companion.InfoParser;

namespace BaxterRobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)
        {

            bool shouldShowHelp = false;
            string robot_info_file = null;
            bool left_arm = false;
            bool right_arm = false;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "left-arm",  n => left_arm = n!=null },
                { "right-arm",  n => right_arm = n!=null },
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

            var robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);
            using (robot_info.Item2)
            {

                ros_csharp_interop.ros_csharp_interop.init_ros(args, "baxter_robotraconteur_driver", true);


                using (var robot = new BaxterRobot(robot_info.Item1, arm_selection, ""))
                {
                    robot._start_robot();
                    using (var node_setup = new ServerNodeSetup(nodename, port, args))
                    {
                        RobotRaconteurNode.s.RegisterService("robot", "com.robotraconteur.robotics.robot", robot);

                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();
                    }
                }
            }

            return 0;

        }
    }
}
