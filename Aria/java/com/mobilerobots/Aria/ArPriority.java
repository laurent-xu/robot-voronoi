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

public class ArPriority {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArPriority(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArPriority obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArPriority(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public static String getPriorityName(ArPriority.Priority priority) {
    return AriaJavaJNI.ArPriority_getPriorityName(priority.swigValue());
  }

  public static ArPriority.Priority getPriorityFromName(String text, SWIGTYPE_p_bool ok) {
    return ArPriority.Priority.swigToEnum(AriaJavaJNI.ArPriority_getPriorityFromName__SWIG_0(text, SWIGTYPE_p_bool.getCPtr(ok)));
  }

  public static ArPriority.Priority getPriorityFromName(String text) {
    return ArPriority.Priority.swigToEnum(AriaJavaJNI.ArPriority_getPriorityFromName__SWIG_1(text));
  }

  public ArPriority() {
    this(AriaJavaJNI.new_ArPriority(), true);
  }

  public final static class Priority {
    public final static ArPriority.Priority INVALID_PRIORITY = new ArPriority.Priority("INVALID_PRIORITY", AriaJavaJNI.ArPriority_INVALID_PRIORITY_get());
    public final static ArPriority.Priority IMPORTANT = new ArPriority.Priority("IMPORTANT");
    public final static ArPriority.Priority BASIC = new ArPriority.Priority("BASIC", AriaJavaJNI.ArPriority_BASIC_get());
    public final static ArPriority.Priority FIRST_PRIORITY = new ArPriority.Priority("FIRST_PRIORITY", AriaJavaJNI.ArPriority_FIRST_PRIORITY_get());
    public final static ArPriority.Priority NORMAL = new ArPriority.Priority("NORMAL");
    public final static ArPriority.Priority INTERMEDIATE = new ArPriority.Priority("INTERMEDIATE", AriaJavaJNI.ArPriority_INTERMEDIATE_get());
    public final static ArPriority.Priority DETAILED = new ArPriority.Priority("DETAILED");
    public final static ArPriority.Priority TRIVIAL = new ArPriority.Priority("TRIVIAL", AriaJavaJNI.ArPriority_TRIVIAL_get());
    public final static ArPriority.Priority ADVANCED = new ArPriority.Priority("ADVANCED", AriaJavaJNI.ArPriority_ADVANCED_get());
    public final static ArPriority.Priority EXPERT = new ArPriority.Priority("EXPERT");
    public final static ArPriority.Priority FACTORY = new ArPriority.Priority("FACTORY");
    public final static ArPriority.Priority CALIBRATION = new ArPriority.Priority("CALIBRATION");
    public final static ArPriority.Priority LAST_PRIORITY = new ArPriority.Priority("LAST_PRIORITY", AriaJavaJNI.ArPriority_LAST_PRIORITY_get());

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static Priority swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + Priority.class + " with value " + swigValue);
    }

    private Priority(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private Priority(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private Priority(String swigName, Priority swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static Priority[] swigValues = { INVALID_PRIORITY, IMPORTANT, BASIC, FIRST_PRIORITY, NORMAL, INTERMEDIATE, DETAILED, TRIVIAL, ADVANCED, EXPERT, FACTORY, CALIBRATION, LAST_PRIORITY };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

  public final static int PRIORITY_COUNT = AriaJavaJNI.ArPriority_PRIORITY_COUNT_get();

}
