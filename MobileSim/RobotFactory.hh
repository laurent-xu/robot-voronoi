/*  
    Copyright (C) 2005, ActivMedia Robotics LLC <http://www.activmedia.com>
    Copyright (C) 2006-2010 MobileRobots, Inc. <http://www.mobilerobots.com>
    Copyright (C) 2011-2015 Adept Technology

    This is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef ROBOT_FACTORY_HH_
#define ROBOT_FACTORY_HH_

#include <string>
#include <set>
#include "RobotInterface.hh"
#include "ArSocket.h"


/** Interface for a class to listen on a port, creating a new robot in the simulator with an attached
 *  EmulatePioneer object for each client that connects.
 */
class RobotFactory
{
public:
  RobotFactory(const std::string& modelName, bool verbose = false, const char
*listenAddress = NULL, const MobileSim::Options *userOpts = NULL);
  virtual ~RobotFactory();
  ArSocket *open(int port, const char *listenAddress = NULL); ///< Open port and return socket, or NULL on error.
  inline std::string getModelName() const { return myModelName; }
//  void setCommandsToIgnore(std::set<int> ig) {
//    myCommandsToIgnore = ig;
//  }
//  void setVerbose(bool v) {
//    myVerbose = v;
//  }
//  void setSRISimCompat(bool v, bool laser) {
//    mySRISimCompat = v;
//    mySRISimLaserCompat = laser;
//  }
//  void setLogPacketsReceived(bool l) {
//    myLogPacketsReceived = l;
//  }
//  void setWarnUnsupportedCommands(bool w) {
//    myWarnUnsupportedCommands = w;
//  }
protected:
  virtual RobotInterface *createRobot(const std::string& modelName, const std::string& requestedRobotName = "") = 0;
  virtual void log_s(const char *msg) { fputs(msg, stderr); }
  virtual void log(const char *fmt, ...)
  {
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
  }
private:
  std::string myModelName;
  int myPort;
  ArSocket myListenSocket;
  //std::set<int> myCommandsToIgnore;
  //bool myVerbose;
  const char *myListenAddress;
  //bool mySRISimCompat;
  //bool mySRISimLaserCompat;
  //bool myLogPacketsReceived;
  void acceptNewClient(unsigned int maxTime);
  ArFunctor1C<RobotFactory, unsigned int> acceptClientCB;
  //bool myWarnUnsupportedCommands;
  const MobileSim::Options *myUserOptions;
};



#endif
