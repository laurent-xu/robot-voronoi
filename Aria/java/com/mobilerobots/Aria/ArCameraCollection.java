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

public class ArCameraCollection {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArCameraCollection(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArCameraCollection obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArCameraCollection(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArCameraCollection() {
    this(AriaJavaJNI.new_ArCameraCollection(), true);
  }

  public boolean addCamera(String cameraName, String cameraType, String displayName, String displayType) {
    return AriaJavaJNI.ArCameraCollection_addCamera(swigCPtr, this, cameraName, cameraType, displayName, displayType);
  }

  public boolean removeCamera(String cameraName) {
    return AriaJavaJNI.ArCameraCollection_removeCamera(swigCPtr, this, cameraName);
  }

  public boolean addCameraCommand(String cameraName, String command, String cameraCommandName, int requestInterval) {
    return AriaJavaJNI.ArCameraCollection_addCameraCommand__SWIG_0(swigCPtr, this, cameraName, command, cameraCommandName, requestInterval);
  }

  public boolean addCameraCommand(String cameraName, String command, String cameraCommandName) {
    return AriaJavaJNI.ArCameraCollection_addCameraCommand__SWIG_1(swigCPtr, this, cameraName, command, cameraCommandName);
  }

  public boolean removeCameraCommand(String cameraName, String command) {
    return AriaJavaJNI.ArCameraCollection_removeCameraCommand(swigCPtr, this, cameraName, command);
  }

  public boolean addParameter(String cameraName, ArCameraParameterSource source, ArConfigArg param) {
    return AriaJavaJNI.ArCameraCollection_addParameter(swigCPtr, this, cameraName, ArCameraParameterSource.getCPtr(source), source, ArConfigArg.getCPtr(param), param);
  }

  public boolean removeParameter(String cameraName, String paramName) {
    return AriaJavaJNI.ArCameraCollection_removeParameter(swigCPtr, this, cameraName, paramName);
  }

  public void getCameraNames(SWIGTYPE_p_std__listT_std__string_t outList) {
    AriaJavaJNI.ArCameraCollection_getCameraNames(swigCPtr, this, SWIGTYPE_p_std__listT_std__string_t.getCPtr(outList));
  }

  public String getCameraType(String cameraName) {
    return AriaJavaJNI.ArCameraCollection_getCameraType(swigCPtr, this, cameraName);
  }

  public String getDisplayName(String cameraName) {
    return AriaJavaJNI.ArCameraCollection_getDisplayName(swigCPtr, this, cameraName);
  }

  public String getDisplayType(String cameraName) {
    return AriaJavaJNI.ArCameraCollection_getDisplayType(swigCPtr, this, cameraName);
  }

  public void getCameraCommands(String cameraName, SWIGTYPE_p_std__listT_std__string_t outList) {
    AriaJavaJNI.ArCameraCollection_getCameraCommands(swigCPtr, this, cameraName, SWIGTYPE_p_std__listT_std__string_t.getCPtr(outList));
  }

  public String getCommandName(String cameraName, String command) {
    return AriaJavaJNI.ArCameraCollection_getCommandName(swigCPtr, this, cameraName, command);
  }

  public int getRequestInterval(String cameraName, String command) {
    return AriaJavaJNI.ArCameraCollection_getRequestInterval(swigCPtr, this, cameraName, command);
  }

  public void getParameterNames(String cameraName, SWIGTYPE_p_std__listT_std__string_t outList) {
    AriaJavaJNI.ArCameraCollection_getParameterNames(swigCPtr, this, cameraName, SWIGTYPE_p_std__listT_std__string_t.getCPtr(outList));
  }

  public boolean getParameter(String cameraName, String parameterName, ArConfigArg paramOut) {
    return AriaJavaJNI.ArCameraCollection_getParameter(swigCPtr, this, cameraName, parameterName, ArConfigArg.getCPtr(paramOut), paramOut);
  }

  public boolean setParameter(String cameraName, ArConfigArg param) {
    return AriaJavaJNI.ArCameraCollection_setParameter(swigCPtr, this, cameraName, ArConfigArg.getCPtr(param), param);
  }

  public boolean exists(String cameraName) {
    return AriaJavaJNI.ArCameraCollection_exists__SWIG_0(swigCPtr, this, cameraName);
  }

  public boolean exists(String cameraName, String command) {
    return AriaJavaJNI.ArCameraCollection_exists__SWIG_1(swigCPtr, this, cameraName, command);
  }

  public boolean parameterExists(String cameraName, String paramName) {
    return AriaJavaJNI.ArCameraCollection_parameterExists(swigCPtr, this, cameraName, paramName);
  }

  public boolean addModifiedCB(ArFunctor functor, ArListPos.Pos position) {
    return AriaJavaJNI.ArCameraCollection_addModifiedCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public boolean addModifiedCB(ArFunctor functor) {
    return AriaJavaJNI.ArCameraCollection_addModifiedCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public boolean removeModifiedCB(ArFunctor functor) {
    return AriaJavaJNI.ArCameraCollection_removeModifiedCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void startUpdate() {
    AriaJavaJNI.ArCameraCollection_startUpdate(swigCPtr, this);
  }

  public void endUpdate() {
    AriaJavaJNI.ArCameraCollection_endUpdate(swigCPtr, this);
  }

  public int lock() {
    return AriaJavaJNI.ArCameraCollection_lock(swigCPtr, this);
  }

  public int tryLock() {
    return AriaJavaJNI.ArCameraCollection_tryLock(swigCPtr, this);
  }

  public int unlock() {
    return AriaJavaJNI.ArCameraCollection_unlock(swigCPtr, this);
  }

}
