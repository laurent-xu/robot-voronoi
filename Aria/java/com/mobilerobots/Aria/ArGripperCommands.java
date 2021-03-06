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

public class ArGripperCommands {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArGripperCommands(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArGripperCommands obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArGripperCommands(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArGripperCommands() {
    this(AriaJavaJNI.new_ArGripperCommands(), true);
  }

  public final static class Commands {
    public final static ArGripperCommands.Commands GRIP_OPEN = new ArGripperCommands.Commands("GRIP_OPEN", AriaJavaJNI.ArGripperCommands_GRIP_OPEN_get());
    public final static ArGripperCommands.Commands GRIP_CLOSE = new ArGripperCommands.Commands("GRIP_CLOSE", AriaJavaJNI.ArGripperCommands_GRIP_CLOSE_get());
    public final static ArGripperCommands.Commands GRIP_STOP = new ArGripperCommands.Commands("GRIP_STOP", AriaJavaJNI.ArGripperCommands_GRIP_STOP_get());
    public final static ArGripperCommands.Commands LIFT_UP = new ArGripperCommands.Commands("LIFT_UP", AriaJavaJNI.ArGripperCommands_LIFT_UP_get());
    public final static ArGripperCommands.Commands LIFT_DOWN = new ArGripperCommands.Commands("LIFT_DOWN", AriaJavaJNI.ArGripperCommands_LIFT_DOWN_get());
    public final static ArGripperCommands.Commands LIFT_STOP = new ArGripperCommands.Commands("LIFT_STOP", AriaJavaJNI.ArGripperCommands_LIFT_STOP_get());
    public final static ArGripperCommands.Commands GRIPPER_STORE = new ArGripperCommands.Commands("GRIPPER_STORE", AriaJavaJNI.ArGripperCommands_GRIPPER_STORE_get());
    public final static ArGripperCommands.Commands GRIPPER_DEPLOY = new ArGripperCommands.Commands("GRIPPER_DEPLOY", AriaJavaJNI.ArGripperCommands_GRIPPER_DEPLOY_get());
    public final static ArGripperCommands.Commands GRIPPER_HALT = new ArGripperCommands.Commands("GRIPPER_HALT", AriaJavaJNI.ArGripperCommands_GRIPPER_HALT_get());
    public final static ArGripperCommands.Commands GRIP_PRESSURE = new ArGripperCommands.Commands("GRIP_PRESSURE", AriaJavaJNI.ArGripperCommands_GRIP_PRESSURE_get());
    public final static ArGripperCommands.Commands LIFT_CARRY = new ArGripperCommands.Commands("LIFT_CARRY", AriaJavaJNI.ArGripperCommands_LIFT_CARRY_get());

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static Commands swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + Commands.class + " with value " + swigValue);
    }

    private Commands(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private Commands(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private Commands(String swigName, Commands swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static Commands[] swigValues = { GRIP_OPEN, GRIP_CLOSE, GRIP_STOP, LIFT_UP, LIFT_DOWN, LIFT_STOP, GRIPPER_STORE, GRIPPER_DEPLOY, GRIPPER_HALT, GRIP_PRESSURE, LIFT_CARRY };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
