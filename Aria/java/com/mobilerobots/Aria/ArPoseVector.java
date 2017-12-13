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

public class ArPoseVector {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArPoseVector(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArPoseVector obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArPoseVector(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArPoseVector() {
    this(AriaJavaJNI.new_ArPoseVector__SWIG_0(), true);
  }

  public ArPoseVector(long n) {
    this(AriaJavaJNI.new_ArPoseVector__SWIG_1(n), true);
  }

  public long size() {
    return AriaJavaJNI.ArPoseVector_size(swigCPtr, this);
  }

  public long capacity() {
    return AriaJavaJNI.ArPoseVector_capacity(swigCPtr, this);
  }

  public void reserve(long n) {
    AriaJavaJNI.ArPoseVector_reserve(swigCPtr, this, n);
  }

  public boolean isEmpty() {
    return AriaJavaJNI.ArPoseVector_isEmpty(swigCPtr, this);
  }

  public void clear() {
    AriaJavaJNI.ArPoseVector_clear(swigCPtr, this);
  }

  public void add(ArPose x) {
    AriaJavaJNI.ArPoseVector_add(swigCPtr, this, ArPose.getCPtr(x), x);
  }

  public ArPose get(int i) {
    return new ArPose(AriaJavaJNI.ArPoseVector_get(swigCPtr, this, i), false);
  }

  public void set(int i, ArPose val) {
    AriaJavaJNI.ArPoseVector_set(swigCPtr, this, i, ArPose.getCPtr(val), val);
  }

}
