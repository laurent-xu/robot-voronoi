/*  
    (C) Copyright 2005, ActivMedia Robotics LLC <http://www.activmedia.com>
    (C) Copyright 2006-2010 MobileRobots, Inc. <http://www.mobilerobots.com>
    (C) Copyright 2011-2015 Adept MobileRobots <http://www.mobilerobots.com>

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

#include <set>
#include <errno.h>

#include "RobotFactory.hh"
#include "RobotInterface.hh"
#include "EmulatePioneer.hh"

#include "MobileSim.hh"
#include "Socket.hh"

RobotFactory::RobotFactory(const std::string& modelName, bool verbose, const char *listenAddress, const MobileSim::Options *userOpts) :
  myModelName(modelName),
  myPort(8101),
  //myVerbose(verbose),
  myListenAddress(listenAddress),
  //mySRISimCompat(false),
  //mySRISimLaserCompat(true),
  //myLogPacketsReceived(false),
  acceptClientCB(this, &RobotFactory::acceptNewClient),
  //myWarnUnsupportedCommands(false)
  myUserOptions(userOpts)
{
}

ArSocket* RobotFactory::open(int port, const char *listenAddress)
{
  if(listenAddress)
    log("RobotFactory: opening port %d for new connections to address %s", port, listenAddress);
  else if(myListenAddress)
    log("RobotFactory: opening port %d for new connections to address %s", port, myListenAddress);
  else
    log("RobotFactory: opening port %d for new connections", port);

  if(myListenSocket.open(port, ArSocket::TCP, listenAddress?listenAddress:myListenAddress)) // == true || mySocket.getError() == ArSocket::NoErr)
  {
    myListenSocket.setNonBlock();
    myListenSocket.setReuseAddress();
    MobileSim::Sockets::addSocketCallback(&myListenSocket, &acceptClientCB, "RobotFactory listening socket (callback to accept clients and create EP objects)");
    //print_debug("RobotFactory socket callback functor is 0x%x", &acceptClientCB);
    return &myListenSocket;
  }
  else  
  {
    fprintf(stderr, "Error %d opening socket on port %d for robot factory: %s\n", myListenSocket.getError(), port, myListenSocket.getErrorStr().c_str());
    log("RobotFactory: error opening socket:");
    log(myListenSocket.getErrorStr().c_str());
    return NULL;
  }
}

void RobotFactory::acceptNewClient(unsigned int /*maxTime*/)
{
  ArSocket *clientSocket = new ArSocket;
  ArTime timer;
  if(!myListenSocket.accept(clientSocket))
  {
    log("RobotFactory: error accepting client:");
    log(myListenSocket.getErrorStr().c_str());
    delete clientSocket;
    return;
  }
  if(!clientSocket->isOpen())
  {
    log("RobotFactory: error accepting client: new client socket not open.");
    delete clientSocket;
    return;
  }
  log("Accepted client. Creating robot...");
  RobotInterface *ri = createRobot(myModelName, clientSocket->getIPString());
  if(!ri)
  {
    log("Robot factory: Error creating new robot. Closing client socket.");
    clientSocket->close();
    delete clientSocket;
    return;
  }
  //print_debug("RobotFactory: new RobotInterface is 0x%x", ri);
  //log(("RobotFactory: created robot interface "));
  // Note, assumes that EmulatePioneer properly deletes client sockets
  // when told to do so (otherwise leaks memory)
  clientSocket->setNonBlock();
  EmulatePioneer *ep = new EmulatePioneer(ri, myModelName, clientSocket, /*deleteOnDisconnect=*/true, /*deleteClientSocketOnDisconnect=*/true, myUserOptions);
  ep->setSimulatorIdentification("MobileSim", MOBILESIM_VERSION);
  //ep->setCommandsToIgnore(myCommandsToIgnore);
  //ep->setVerbose(myVerbose);
  //ep->setSRISimCompat(mySRISimCompat, mySRISimLaserCompat);
  //ep->setLogPacketsReceived(myLogPacketsReceived);
  //ep->setWarnUnsupportedCommands(myWarnUnsupportedCommands);
} 


RobotFactory::~RobotFactory() 
{
}
