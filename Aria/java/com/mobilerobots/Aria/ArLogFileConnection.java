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

public class ArLogFileConnection extends ArDeviceConnection {
  private transient long swigCPtr;

  public ArLogFileConnection(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.ArLogFileConnection_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArLogFileConnection obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArLogFileConnection(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ArLogFileConnection() {
    this(AriaJavaJNI.new_ArLogFileConnection(), true);
  }

  public int open(String fname) {
    return AriaJavaJNI.ArLogFileConnection_open__SWIG_0(swigCPtr, this, fname);
  }

  public int open() {
    return AriaJavaJNI.ArLogFileConnection_open__SWIG_1(swigCPtr, this);
  }

  public void setLogFile(String fname) {
    AriaJavaJNI.ArLogFileConnection_setLogFile__SWIG_0(swigCPtr, this, fname);
  }

  public void setLogFile() {
    AriaJavaJNI.ArLogFileConnection_setLogFile__SWIG_1(swigCPtr, this);
  }

  public boolean openSimple() {
    return AriaJavaJNI.ArLogFileConnection_openSimple(swigCPtr, this);
  }

  public int getStatus() {
    return AriaJavaJNI.ArLogFileConnection_getStatus(swigCPtr, this);
  }

  public boolean close() {
    return AriaJavaJNI.ArLogFileConnection_close(swigCPtr, this);
  }

  public int read(String data, long size, long msWait) {
    return AriaJavaJNI.ArLogFileConnection_read__SWIG_0(swigCPtr, this, data, size, msWait);
  }

  public int read(String data, long size) {
    return AriaJavaJNI.ArLogFileConnection_read__SWIG_1(swigCPtr, this, data, size);
  }

  public int write(String data, long size) {
    return AriaJavaJNI.ArLogFileConnection_write(swigCPtr, this, data, size);
  }

  public String getOpenMessage(int messageNumber) {
    return AriaJavaJNI.ArLogFileConnection_getOpenMessage(swigCPtr, this, messageNumber);
  }

  public ArTime getTimeRead(int index) {
    return new ArTime(AriaJavaJNI.ArLogFileConnection_getTimeRead(swigCPtr, this, index), true);
  }

  public boolean isTimeStamping() {
    return AriaJavaJNI.ArLogFileConnection_isTimeStamping(swigCPtr, this);
  }

  public String getLogFile() {
    return AriaJavaJNI.ArLogFileConnection_getLogFile(swigCPtr, this);
  }

  public int internalOpen() {
    return AriaJavaJNI.ArLogFileConnection_internalOpen(swigCPtr, this);
  }

  public void setMyPose(ArPose value) {
    AriaJavaJNI.ArLogFileConnection_myPose_set(swigCPtr, this, ArPose.getCPtr(value), value);
  }

  public ArPose getMyPose() {
    long cPtr = AriaJavaJNI.ArLogFileConnection_myPose_get(swigCPtr, this);
    return (cPtr == 0) ? null : new ArPose(cPtr, false);
  }

  public void setHavePose(boolean value) {
    AriaJavaJNI.ArLogFileConnection_havePose_set(swigCPtr, this, value);
  }

  public boolean getHavePose() {
    return AriaJavaJNI.ArLogFileConnection_havePose_get(swigCPtr, this);
  }

  public void setMyName(String value) {
    AriaJavaJNI.ArLogFileConnection_myName_set(swigCPtr, this, value);
  }

  public String getMyName() {
    return AriaJavaJNI.ArLogFileConnection_myName_get(swigCPtr, this);
  }

  public void setMyType(String value) {
    AriaJavaJNI.ArLogFileConnection_myType_set(swigCPtr, this, value);
  }

  public String getMyType() {
    return AriaJavaJNI.ArLogFileConnection_myType_get(swigCPtr, this);
  }

  public void setMySubtype(String value) {
    AriaJavaJNI.ArLogFileConnection_mySubtype_set(swigCPtr, this, value);
  }

  public String getMySubtype() {
    return AriaJavaJNI.ArLogFileConnection_mySubtype_get(swigCPtr, this);
  }

  public final static class Open {
    public final static ArLogFileConnection.Open OPEN_FILE_NOT_FOUND = new ArLogFileConnection.Open("OPEN_FILE_NOT_FOUND", AriaJavaJNI.ArLogFileConnection_OPEN_FILE_NOT_FOUND_get());
    public final static ArLogFileConnection.Open OPEN_NOT_A_LOG_FILE = new ArLogFileConnection.Open("OPEN_NOT_A_LOG_FILE");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static Open swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + Open.class + " with value " + swigValue);
    }

    private Open(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private Open(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private Open(String swigName, Open swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static Open[] swigValues = { OPEN_FILE_NOT_FOUND, OPEN_NOT_A_LOG_FILE };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
