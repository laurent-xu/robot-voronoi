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
#include "Aria.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include "pathbuilder.hh"
#define TO_RAD(X) (X * M_PI / 180.)

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

void turn(const ArPose& destination, ArRobot& robot)
{
  double rot_vel = 20.;
  double Te = 1000;

  double var_angle = robot.findDeltaHeadingTo(destination);
  double distance = robot.findDistanceTo(destination);
  double dst_x = destination.getX();
  double dst_y = destination.getY();


  std::cout << std::endl << "Start turn " << std::endl;
  while (abs(var_angle) > 4.)
  {
    rot_vel = std::min(rot_vel, std::abs(var_angle / 4.));
    double actual_rot_vel = rot_vel * (var_angle < 0. ? -1. : 1.);
    double time_rotation = std::abs(var_angle / actual_rot_vel * Te);

    std::cout << "To do angle: " << var_angle << std::endl
              << "angular speed: " << actual_rot_vel << std::endl
              << "time of rotation: " << time_rotation / Te << std::endl;

    robot.lock();
    robot.setVel(0.);
    robot.setRotVel(actual_rot_vel);
    robot.unlock();
    ArUtil::sleep(time_rotation);

    var_angle = robot.findDeltaHeadingTo(destination);
  }
}

void go_to(ArPose destination, const ArPose& source, double source_angle,
           ArRobot& robot)
{
  double Te = 1000;
  double vel = 1000.;

  destination -= source;
  auto cos_source = cos(-TO_RAD(source_angle));
  auto sin_source = sin(-TO_RAD(source_angle));
  auto dst_x = destination.getX();
  auto dst_y = destination.getY();
  destination = ArPose(dst_x * cos_source - dst_y * sin_source,
                       dst_y * cos_source + dst_x * sin_source);
  double distance = robot.findDistanceTo(destination);
  std::cout << "Relative destination " << destination.getX()
            << " " << destination.getY() << std::endl;

  while (distance > 100.)
  {
    vel = std::min(vel, distance / 4.);

    std::cout << "Wait: " << 1. / 4. << std::endl;
    robot.lock();
    robot.setVel(0.);
    robot.setRotVel(0.);
    robot.unlock();
    ArUtil::sleep(Te / 4.);

    turn(destination, robot);

    std::cout << "Wait: " << 1. / 4. << std::endl;
    robot.lock();
    robot.setVel(0.);
    robot.setRotVel(0.);
    robot.unlock();
    ArUtil::sleep(Te / 4.);

    distance = std::min(distance, 1000.);
    double time_move = distance / vel * Te;
    std::cout << std::endl << "Start move " << std::endl;
    std::cout << "To do distance: " << distance << std::endl
      << "speed: " << vel << std::endl
      << "time of movement: " << time_move << std::endl;

    robot.lock();
    robot.setVel(vel);
    robot.setRotVel(0.);
    robot.unlock();
    ArUtil::sleep(time_move);

    distance = robot.findDistanceTo(destination);
    std::cout << "REAL DISTANCE" << distance << std::endl;
  }
}


int main(int argc, char **argv)
{
  Aria::init();
  ArRobot robot;

  ArArgumentParser parser(&argc, argv);
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleMotionCommands: Could not connect "
                              "to the robot.");
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

  std::ifstream ifs("planification.txt");
  if (ifs)
  {
    double src_x, src_y, src_angle, dst_x, dst_y;
    std::string map_name;

    ifs >> src_x >> src_y >> src_angle >> dst_x >> dst_y >> map_name;

    std::cout << "Origin of the robot: " << src_x << " " << src_y << std::endl
              << "Original angle: " << src_angle << std::endl
              << "Destination: " << dst_x << dst_y << std::endl
              << "Map name: " << map_name << std::endl;

    ArPose source(src_x, src_y);
    ArPose destination(dst_x, dst_y);

    ArMap map;
    if (!map.readFile(map_name.c_str()))
    {
      std::cerr << "Impossible de lire " << map_name << std::endl;
      return 1;
    }

    PathBuilder path_builder(map);

    robot.lock();
    robot.enableMotors();
    robot.unlock();
    ArUtil::sleep(1000);

    auto interest_points = path_builder.get_path(source, destination);
    for (auto& p: interest_points)
      go_to(p, source, src_angle, robot);

    ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
    Aria::exit(0);
    return 0;
  }
  else
  {
    std::cerr << "Le fichier planification.txt ne s'est pas "
                 "ouvert correctement" << std::endl;
  }
}
