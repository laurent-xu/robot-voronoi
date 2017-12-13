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

package com.mobilerobots.Aria;

public class ArActionIRs extends ArAction {
  private transient long swigCPtr;

  public ArActionIRs(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.ArActionIRs_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArActionIRs obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArActionIRs(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ArActionIRs(String name, double backOffSpeed, int backOffTime, int turnTime, boolean setMaximums) {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_0(name, backOffSpeed, backOffTime, turnTime, setMaximums), true);
  }

  public ArActionIRs(String name, double backOffSpeed, int backOffTime, int turnTime) {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_1(name, backOffSpeed, backOffTime, turnTime), true);
  }

  public ArActionIRs(String name, double backOffSpeed, int backOffTime) {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_2(name, backOffSpeed, backOffTime), true);
  }

  public ArActionIRs(String name, double backOffSpeed) {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_3(name, backOffSpeed), true);
  }

  public ArActionIRs(String name) {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_4(name), true);
  }

  public ArActionIRs() {
    this(AriaJavaJNI.new_ArActionIRs__SWIG_5(), true);
  }

  public ArActionDesired fire(ArActionDesired currentDesired) {
    long cPtr = AriaJavaJNI.ArActionIRs_fire(swigCPtr, this, ArActionDesired.getCPtr(currentDesired), currentDesired);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

  public void setRobot(ArRobot robot) {
    AriaJavaJNI.ArActionIRs_setRobot(swigCPtr, this, ArRobot.getCPtr(robot), robot);
  }

  public ArActionDesired getDesired() {
    long cPtr = AriaJavaJNI.ArActionIRs_getDesired(swigCPtr, this);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

}
