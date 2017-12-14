/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include <Aria.h>
#include <cmath>
#include <iostream>

/** @example simpleMotionCommands.cpp example showing how to connect and send
 * basic motion commands to the robot
 *
 * ARIA provides two levels of robot motion control, direct motion commands, and
 * actions. This example shows direct motion commands. See actionExample.cpp,
 * actionGroupExample.cpp, and others for examples on how to use actions.
 * Actions provide a more modular way of performing more complex motion
 * behaviors than the simple imperitive style used here.  
 *
 * See the ArRobot class documentation, as well as the overview of robot motion,
 * for more information.
 *
 * WARNING: this program does no sensing or avoiding of obstacles, the robot WILL
 * collide with any objects in the way!   Make sure the robot has about 2-3
 * meters of free space around it before starting the program.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

void goTo(double dst_x, double dst_y, ArRobot& robot)
{
	ArPose destination(dst_x, dst_y);
	double rot_vel = 10.;
	double Te = 1000;
	double vel = 250.;
	auto src = robot.getPose();	
	double src_x = src.getX();
	double src_y = src.getY();
	double src_angle = src.getTh();
	double var_x = dst_x - src_x;
	double var_y = dst_y - src_y;
	double var_distance = sqrt(var_x * var_x + var_y * var_y);
	double var_angle = robot.findDeltaHeadingTo(destination);
	std::cerr << src_x << " " << src_y << " " << dst_x << " " << dst_y << " " << var_distance << " " << var_angle << std::endl;
	robot.lock();
	robot.setVel(0.);
	double actual_rot_vel = var_angle / abs(var_angle) * rot_vel;
	std::cerr << actual_rot_vel << std::endl;
	robot.setRotVel(actual_rot_vel);
	robot.unlock();
	ArUtil::sleep(abs(var_angle) / abs(actual_rot_vel) * Te);
	robot.lock();
	robot.setVel(0.);
	robot.setRotVel(0);
	robot.unlock();
	robot.lock();
	ArUtil::sleep(Te * 2);
	robot.setVel(vel);
	robot.setRotVel(0);
	robot.unlock();
	ArUtil::sleep(var_distance / vel * Te);
	robot.lock();
	robot.setVel(0.);
	robot.setRotVel(0);
	robot.unlock();
	ArUtil::sleep(Te * 2);
	std::cerr << "test" << robot.findDeltaHeadingTo(destination) << std::endl;
}

int main(int argc, char **argv)
{

  Aria::init();
  ArRobot robot;

  ArArgumentParser parser(&argc, argv);
  // ArRobotConnector connects to the robot, get some initial data from it such as type and name,
  // and then loads parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }
  
  robot.runAsync(true);

  robot.lock();
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
  ArUtil::sleep(3000);

  // Set forward velocity to 50 mm/s
  ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
  robot.lock();
  robot.enableMotors();
  robot.unlock();
  ArUtil::sleep(1000);

  goTo(100, 100, robot);

  ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
  Aria::exit(0);
  return 0;
}
