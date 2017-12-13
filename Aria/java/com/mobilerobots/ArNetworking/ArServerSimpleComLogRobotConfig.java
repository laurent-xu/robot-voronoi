/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016-2017 Omron Adept Technologies, Inc.

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
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 3.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.mobilerobots.ArNetworking;
import com.mobilerobots.Aria.*;
public class ArServerSimpleComLogRobotConfig {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArServerSimpleComLogRobotConfig(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArServerSimpleComLogRobotConfig obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArServerSimpleComLogRobotConfig(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArServerSimpleComLogRobotConfig(ArServerHandlerCommands commands, ArRobot robot, SWIGTYPE_p_ArServerHandlerPopup popupHandler) {
    this(ArNetworkingJavaJNI.new_ArServerSimpleComLogRobotConfig__SWIG_0(ArServerHandlerCommands.getCPtr(commands), commands, ArRobot.getCPtr(robot), robot, SWIGTYPE_p_ArServerHandlerPopup.getCPtr(popupHandler)), true);
  }

  public ArServerSimpleComLogRobotConfig(ArServerHandlerCommands commands, ArRobot robot) {
    this(ArNetworkingJavaJNI.new_ArServerSimpleComLogRobotConfig__SWIG_1(ArServerHandlerCommands.getCPtr(commands), commands, ArRobot.getCPtr(robot), robot), true);
  }

  public void logConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_logConfig(swigCPtr, this);
  }

  public void logMovementConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_logMovementConfig(swigCPtr, this);
  }

  public void logOrigConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_logOrigConfig(swigCPtr, this);
  }

  public void popupConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_popupConfig(swigCPtr, this);
  }

  public void popupOrigConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_popupOrigConfig(swigCPtr, this);
  }

  public void popupMovementConfig() {
    ArNetworkingJavaJNI.ArServerSimpleComLogRobotConfig_popupMovementConfig(swigCPtr, this);
  }

}
