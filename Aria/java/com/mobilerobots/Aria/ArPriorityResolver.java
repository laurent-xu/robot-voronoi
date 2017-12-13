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

public class ArPriorityResolver extends ArResolver {
  private transient long swigCPtr;

  public ArPriorityResolver(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.ArPriorityResolver_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArPriorityResolver obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArPriorityResolver(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ArPriorityResolver() {
    this(AriaJavaJNI.new_ArPriorityResolver(), true);
  }

  public ArActionDesired resolve(SWIGTYPE_p_std__multimapT_int_ArAction_p_t actions, ArRobot robot, boolean logActions) {
    long cPtr = AriaJavaJNI.ArPriorityResolver_resolve__SWIG_0(swigCPtr, this, SWIGTYPE_p_std__multimapT_int_ArAction_p_t.getCPtr(actions), ArRobot.getCPtr(robot), robot, logActions);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

  public ArActionDesired resolve(SWIGTYPE_p_std__multimapT_int_ArAction_p_t actions, ArRobot robot) {
    long cPtr = AriaJavaJNI.ArPriorityResolver_resolve__SWIG_1(swigCPtr, this, SWIGTYPE_p_std__multimapT_int_ArAction_p_t.getCPtr(actions), ArRobot.getCPtr(robot), robot);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

}
