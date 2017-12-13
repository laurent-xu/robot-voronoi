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
public class ArServerFileFromClient {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArServerFileFromClient(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArServerFileFromClient obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArServerFileFromClient(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArServerFileFromClient(ArServerBase server, String topDir, String tempDir) {
    this(ArNetworkingJavaJNI.new_ArServerFileFromClient(ArServerBase.getCPtr(server), server, topDir, tempDir), true);
  }

  public void putFile(ArServerClient client, ArNetPacket packet) {
    ArNetworkingJavaJNI.ArServerFileFromClient_putFile(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void putFileInterleaved(ArServerClient client, ArNetPacket packet) {
    ArNetworkingJavaJNI.ArServerFileFromClient_putFileInterleaved(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void putFileWithTimestamp(ArServerClient client, ArNetPacket packet) {
    ArNetworkingJavaJNI.ArServerFileFromClient_putFileWithTimestamp(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void putFileWithTimestampInterleaved(ArServerClient client, ArNetPacket packet) {
    ArNetworkingJavaJNI.ArServerFileFromClient_putFileWithTimestampInterleaved(swigCPtr, this, ArServerClient.getCPtr(client), client, ArNetPacket.getCPtr(packet), packet);
  }

  public void addPreMoveCallback(ArFunctor functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArServerFileFromClient_addPreMoveCallback__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addPreMoveCallback(ArFunctor functor) {
    ArNetworkingJavaJNI.ArServerFileFromClient_addPreMoveCallback__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remPreMoveCallback(ArFunctor functor) {
    ArNetworkingJavaJNI.ArServerFileFromClient_remPreMoveCallback(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void addPostMoveCallback(ArFunctor functor, ArListPos.Pos position) {
    ArNetworkingJavaJNI.ArServerFileFromClient_addPostMoveCallback__SWIG_0(swigCPtr, this, ArFunctor.getCPtr(functor), functor, position.swigValue());
  }

  public void addPostMoveCallback(ArFunctor functor) {
    ArNetworkingJavaJNI.ArServerFileFromClient_addPostMoveCallback__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void remPostMoveCallback(ArFunctor functor) {
    ArNetworkingJavaJNI.ArServerFileFromClient_remPostMoveCallback(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public String getMovingFileName() {
    return ArNetworkingJavaJNI.ArServerFileFromClient_getMovingFileName(swigCPtr, this);
  }

}
