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
public class ArServerUserInfo {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArServerUserInfo(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArServerUserInfo obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArServerUserInfo(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArServerUserInfo(String baseDirectory) {
    this(ArNetworkingJavaJNI.new_ArServerUserInfo__SWIG_0(baseDirectory), true);
  }

  public ArServerUserInfo() {
    this(ArNetworkingJavaJNI.new_ArServerUserInfo__SWIG_1(), true);
  }

  public boolean readFile(String fileName) {
    return ArNetworkingJavaJNI.ArServerUserInfo_readFile(swigCPtr, this, fileName);
  }

  public void setBaseDirectory(String baseDirectory) {
    ArNetworkingJavaJNI.ArServerUserInfo_setBaseDirectory(swigCPtr, this, baseDirectory);
  }

  public boolean matchUserPassword(String user, SWIGTYPE_p_unsigned_char password, String passwordKey, String serverKey, boolean logFailureVerbosely) {
    return ArNetworkingJavaJNI.ArServerUserInfo_matchUserPassword__SWIG_0(swigCPtr, this, user, SWIGTYPE_p_unsigned_char.getCPtr(password), passwordKey, serverKey, logFailureVerbosely);
  }

  public boolean matchUserPassword(String user, SWIGTYPE_p_unsigned_char password, String passwordKey, String serverKey) {
    return ArNetworkingJavaJNI.ArServerUserInfo_matchUserPassword__SWIG_1(swigCPtr, this, user, SWIGTYPE_p_unsigned_char.getCPtr(password), passwordKey, serverKey);
  }

  public boolean doNotUse() {
    return ArNetworkingJavaJNI.ArServerUserInfo_doNotUse(swigCPtr, this);
  }

  public SWIGTYPE_p_std__setT_std__string_ArStrCaseCmpOp_t getUsersGroups(String user) {
    return new SWIGTYPE_p_std__setT_std__string_ArStrCaseCmpOp_t(ArNetworkingJavaJNI.ArServerUserInfo_getUsersGroups(swigCPtr, this, user), true);
  }

  public void logUsers() {
    ArNetworkingJavaJNI.ArServerUserInfo_logUsers(swigCPtr, this);
  }

}
