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
public class ArClientHandlerConfig {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArClientHandlerConfig(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArClientHandlerConfig obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArClientHandlerConfig(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArClientHandlerConfig(ArClientBase client, boolean ignoreBounds, String robotName, String logPrefix) {
    this(ArNetworkingJavaJNI.new_ArClientHandlerConfig__SWIG_0(ArClientBase.getCPtr(client), client, ignoreBounds, robotName, logPrefix), true);
  }

  public ArClientHandlerConfig(ArClientBase client, boolean ignoreBounds, String robotName) {
    this(ArNetworkingJavaJNI.new_ArClientHandlerConfig__SWIG_1(ArClientBase.getCPtr(client), client, ignoreBounds, robotName), true);
  }

  public ArClientHandlerConfig(ArClientBase client, boolean ignoreBounds) {
    this(ArNetworkingJavaJNI.new_ArClientHandlerConfig__SWIG_2(ArClientBase.getCPtr(client), client, ignoreBounds), true);
  }

  public ArClientHandlerConfig(ArClientBase client) {
    this(ArNetworkingJavaJNI.new_ArClientHandlerConfig__SWIG_3(ArClientBase.getCPtr(client), client), true);
  }

  public void requestConfigFromServer() {
    ArNetworkingJavaJNI.ArClientHandlerConfig_requestConfigFromServer(swigCPtr, this);
  }

  public void reloadConfigOnServer() {
    ArNetworkingJavaJNI.ArClientHandlerConfig_reloadConfigOnServer(swigCPtr, this);
  }

  public ArConfig getConfigCopy() {
    return new ArConfig(ArNetworkingJavaJNI.ArClientHandlerConfig_getConfigCopy(swigCPtr, this), true);
  }

  public void addGotConfigCB(ArFunctor functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotConfigCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addGotConfigCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotConfigCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remGotConfigCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_remGotConfigCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addSaveConfigSucceededCB(ArFunctor functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addSaveConfigSucceededCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addSaveConfigSucceededCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addSaveConfigSucceededCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remSaveConfigSucceededCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_remSaveConfigSucceededCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addSaveConfigFailedCB(ArFunctor1_CString functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addSaveConfigFailedCB__SWIG_0(swigCPtr, this, ArFunctor1_CString.getCPtr(functor), functor, position.swigValue());
  }

  public void addSaveConfigFailedCB(ArFunctor1_CString functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addSaveConfigFailedCB__SWIG_1(swigCPtr, this, ArFunctor1_CString.getCPtr(functor), functor);
  }

  public void remSaveConfigFailedCB(ArFunctor1_CString functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_remSaveConfigFailedCB(swigCPtr, this, ArFunctor1_CString.getCPtr(functor), functor);
  }

  public boolean haveGottenConfig() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_haveGottenConfig(swigCPtr, this);
  }

  public void saveConfigToServer() {
    ArNetworkingJavaJNI.ArClientHandlerConfig_saveConfigToServer__SWIG_0(swigCPtr, this);
  }

  public void saveConfigToServer(ArConfig config, SWIGTYPE_p_std__setT_std__string_ArStrCaseCmpOp_t ignoreTheseSections) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_saveConfigToServer__SWIG_1(swigCPtr, this, ArConfig.getCPtr(config), config, SWIGTYPE_p_std__setT_std__string_ArStrCaseCmpOp_t.getCPtr(ignoreTheseSections));
  }

  public void saveConfigToServer(ArConfig config) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_saveConfigToServer__SWIG_2(swigCPtr, this, ArConfig.getCPtr(config), config);
  }

  public boolean haveRequestedDefaults() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_haveRequestedDefaults(swigCPtr, this);
  }

  public boolean haveGottenDefaults() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_haveGottenDefaults(swigCPtr, this);
  }

  public boolean canRequestDefaults() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_canRequestDefaults(swigCPtr, this);
  }

  public boolean requestDefaultConfigFromServer() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_requestDefaultConfigFromServer(swigCPtr, this);
  }

  public ArConfig getDefaultConfig() {
    long cPtr = ArNetworkingJavaJNI.ArClientHandlerConfig_getDefaultConfig(swigCPtr, this);
    return (cPtr == 0) ? null : new ArConfig(cPtr, false);
  }

  public boolean requestConfigDefaults() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_requestConfigDefaults(swigCPtr, this);
  }

  public boolean requestSectionDefaults(String section) {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_requestSectionDefaults(swigCPtr, this, section);
  }

  public void addGotConfigDefaultsCB(ArFunctor functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotConfigDefaultsCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addGotConfigDefaultsCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotConfigDefaultsCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remGotConfigDefaultsCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_remGotConfigDefaultsCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public boolean isLastEditablePriorityAvailable() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_isLastEditablePriorityAvailable(swigCPtr, this);
  }

  public boolean requestLastEditablePriorityFromServer() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_requestLastEditablePriorityFromServer(swigCPtr, this);
  }

  public boolean haveGottenLastEditablePriority() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_haveGottenLastEditablePriority(swigCPtr, this);
  }

  public ArPriority.Priority getLastEditablePriority() {
    return ArPriority.Priority.swigToEnum(ArNetworkingJavaJNI.ArClientHandlerConfig_getLastEditablePriority(swigCPtr, this));
  }

  public void addGotLastEditablePriorityCB(ArFunctor functor, int position) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotLastEditablePriorityCB__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position);
  }

  public void addGotLastEditablePriorityCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_addGotLastEditablePriorityCB__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remGotLastEditablePriorityCB(ArFunctor functor) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_remGotLastEditablePriorityCB(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public ArConfig getConfig() {
    long cPtr = ArNetworkingJavaJNI.ArClientHandlerConfig_getConfig(swigCPtr, this);
    return (cPtr == 0) ? null : new ArConfig(cPtr, false);
  }

  public int lock() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_lock(swigCPtr, this);
  }

  public int tryLock() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_tryLock(swigCPtr, this);
  }

  public int unlock() {
    return ArNetworkingJavaJNI.ArClientHandlerConfig_unlock(swigCPtr, this);
  }

  public void setQuiet(boolean isQuiet) {
    ArNetworkingJavaJNI.ArClientHandlerConfig_setQuiet(swigCPtr, this, isQuiet);
  }

}
