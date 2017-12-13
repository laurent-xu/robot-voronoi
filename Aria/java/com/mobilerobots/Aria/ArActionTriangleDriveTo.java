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

public class ArActionTriangleDriveTo extends ArAction {
  private transient long swigCPtr;

  public ArActionTriangleDriveTo(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.ArActionTriangleDriveTo_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArActionTriangleDriveTo obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArActionTriangleDriveTo(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ArActionTriangleDriveTo(String name, double finalDistFromVertex, double approachDistFromVertex, double speed, double closeDist, double acquireTurnSpeed) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_0(name, finalDistFromVertex, approachDistFromVertex, speed, closeDist, acquireTurnSpeed), true);
  }

  public ArActionTriangleDriveTo(String name, double finalDistFromVertex, double approachDistFromVertex, double speed, double closeDist) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_1(name, finalDistFromVertex, approachDistFromVertex, speed, closeDist), true);
  }

  public ArActionTriangleDriveTo(String name, double finalDistFromVertex, double approachDistFromVertex, double speed) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_2(name, finalDistFromVertex, approachDistFromVertex, speed), true);
  }

  public ArActionTriangleDriveTo(String name, double finalDistFromVertex, double approachDistFromVertex) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_3(name, finalDistFromVertex, approachDistFromVertex), true);
  }

  public ArActionTriangleDriveTo(String name, double finalDistFromVertex) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_4(name, finalDistFromVertex), true);
  }

  public ArActionTriangleDriveTo(String name) {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_5(name), true);
  }

  public ArActionTriangleDriveTo() {
    this(AriaJavaJNI.new_ArActionTriangleDriveTo__SWIG_6(), true);
  }

  public void setAcquire(boolean acquire) {
    AriaJavaJNI.ArActionTriangleDriveTo_setAcquire__SWIG_0(swigCPtr, this, acquire);
  }

  public void setAcquire() {
    AriaJavaJNI.ArActionTriangleDriveTo_setAcquire__SWIG_1(swigCPtr, this);
  }

  public boolean getAcquire() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getAcquire(swigCPtr, this);
  }

  public void setFinalDistFromVertex(double dist) {
    AriaJavaJNI.ArActionTriangleDriveTo_setFinalDistFromVertex(swigCPtr, this, dist);
  }

  public double getFinalDistFromVertex() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getFinalDistFromVertex(swigCPtr, this);
  }

  public void setTriangleParams(double line1Length, double angleBetween, double line2Length) {
    AriaJavaJNI.ArActionTriangleDriveTo_setTriangleParams__SWIG_0(swigCPtr, this, line1Length, angleBetween, line2Length);
  }

  public void setTriangleParams(double line1Length, double angleBetween) {
    AriaJavaJNI.ArActionTriangleDriveTo_setTriangleParams__SWIG_1(swigCPtr, this, line1Length, angleBetween);
  }

  public void setTriangleParams(double line1Length) {
    AriaJavaJNI.ArActionTriangleDriveTo_setTriangleParams__SWIG_2(swigCPtr, this, line1Length);
  }

  public void setTriangleParams() {
    AriaJavaJNI.ArActionTriangleDriveTo_setTriangleParams__SWIG_3(swigCPtr, this);
  }

  public void setParameters(double finalDistFromVertex, double approachDistFromVertex, double speed, double closeDist, double acquireTurnSpeed) {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_0(swigCPtr, this, finalDistFromVertex, approachDistFromVertex, speed, closeDist, acquireTurnSpeed);
  }

  public void setParameters(double finalDistFromVertex, double approachDistFromVertex, double speed, double closeDist) {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_1(swigCPtr, this, finalDistFromVertex, approachDistFromVertex, speed, closeDist);
  }

  public void setParameters(double finalDistFromVertex, double approachDistFromVertex, double speed) {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_2(swigCPtr, this, finalDistFromVertex, approachDistFromVertex, speed);
  }

  public void setParameters(double finalDistFromVertex, double approachDistFromVertex) {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_3(swigCPtr, this, finalDistFromVertex, approachDistFromVertex);
  }

  public void setParameters(double finalDistFromVertex) {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_4(swigCPtr, this, finalDistFromVertex);
  }

  public void setParameters() {
    AriaJavaJNI.ArActionTriangleDriveTo_setParameters__SWIG_5(swigCPtr, this);
  }

  public boolean getAdjustVertex() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getAdjustVertex(swigCPtr, this);
  }

  public void setAdjustVertex(boolean adjustVertex) {
    AriaJavaJNI.ArActionTriangleDriveTo_setAdjustVertex(swigCPtr, this, adjustVertex);
  }

  public void setVertexOffset(int localXOffset, int localYOffset, double thOffset) {
    AriaJavaJNI.ArActionTriangleDriveTo_setVertexOffset(swigCPtr, this, localXOffset, localYOffset, thOffset);
  }

  public void setUseLegacyVertexOffset(boolean useLegacyVertexOffset) {
    AriaJavaJNI.ArActionTriangleDriveTo_setUseLegacyVertexOffset(swigCPtr, this, useLegacyVertexOffset);
  }

  public boolean getUseLegacyVertexOffset() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getUseLegacyVertexOffset(swigCPtr, this);
  }

  public boolean getGotoVertex() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getGotoVertex(swigCPtr, this);
  }

  public void setGotoVertex(boolean gotoVertex) {
    AriaJavaJNI.ArActionTriangleDriveTo_setGotoVertex(swigCPtr, this, gotoVertex);
  }

  public void setIgnoreTriangleDist(double dist, boolean useIgnoreInGotoVertexMode) {
    AriaJavaJNI.ArActionTriangleDriveTo_setIgnoreTriangleDist__SWIG_0(swigCPtr, this, dist, useIgnoreInGotoVertexMode);
  }

  public void setIgnoreTriangleDist(double dist) {
    AriaJavaJNI.ArActionTriangleDriveTo_setIgnoreTriangleDist__SWIG_1(swigCPtr, this, dist);
  }

  public void setIgnoreTriangleDist() {
    AriaJavaJNI.ArActionTriangleDriveTo_setIgnoreTriangleDist__SWIG_2(swigCPtr, this);
  }

  public double getIgnoreTriangleDist() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getIgnoreTriangleDist(swigCPtr, this);
  }

  public boolean getUseIgnoreInGotoVertexMode() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getUseIgnoreInGotoVertexMode(swigCPtr, this);
  }

  public void setVertexUnseenStopMSecs(int vertexUnseenStopMSecs) {
    AriaJavaJNI.ArActionTriangleDriveTo_setVertexUnseenStopMSecs__SWIG_0(swigCPtr, this, vertexUnseenStopMSecs);
  }

  public void setVertexUnseenStopMSecs() {
    AriaJavaJNI.ArActionTriangleDriveTo_setVertexUnseenStopMSecs__SWIG_1(swigCPtr, this);
  }

  public int getVertexUnseenStopMSecs() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getVertexUnseenStopMSecs(swigCPtr, this);
  }

  public void setMaxDistBetweenLinePoints(int maxDistBetweenLinePoints) {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxDistBetweenLinePoints__SWIG_0(swigCPtr, this, maxDistBetweenLinePoints);
  }

  public void setMaxDistBetweenLinePoints() {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxDistBetweenLinePoints__SWIG_1(swigCPtr, this);
  }

  public void setMaxLateralDist(int maxLateralDist) {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxLateralDist__SWIG_0(swigCPtr, this, maxLateralDist);
  }

  public void setMaxLateralDist() {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxLateralDist__SWIG_1(swigCPtr, this);
  }

  public void setMaxAngleMisalignment(int maxAngleMisalignment) {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxAngleMisalignment__SWIG_0(swigCPtr, this, maxAngleMisalignment);
  }

  public void setMaxAngleMisalignment() {
    AriaJavaJNI.ArActionTriangleDriveTo_setMaxAngleMisalignment__SWIG_1(swigCPtr, this);
  }

  public ArActionTriangleDriveTo.State getState() {
    return ArActionTriangleDriveTo.State.swigToEnum(AriaJavaJNI.ArActionTriangleDriveTo_getState(swigCPtr, this));
  }

  public boolean getVertexSeen() {
    return AriaJavaJNI.ArActionTriangleDriveTo_getVertexSeen(swigCPtr, this);
  }

  public void setLineFinder(ArLineFinder lineFinder) {
    AriaJavaJNI.ArActionTriangleDriveTo_setLineFinder(swigCPtr, this, ArLineFinder.getCPtr(lineFinder), lineFinder);
  }

  public ArLineFinder getLineFinder() {
    long cPtr = AriaJavaJNI.ArActionTriangleDriveTo_getLineFinder(swigCPtr, this);
    return (cPtr == 0) ? null : new ArLineFinder(cPtr, false);
  }

  public void setLogging(boolean logging) {
    AriaJavaJNI.ArActionTriangleDriveTo_setLogging__SWIG_0(swigCPtr, this, logging);
  }

  public boolean setLogging() {
    return AriaJavaJNI.ArActionTriangleDriveTo_setLogging__SWIG_1(swigCPtr, this);
  }

  public void activate() {
    AriaJavaJNI.ArActionTriangleDriveTo_activate(swigCPtr, this);
  }

  public void deactivate() {
    AriaJavaJNI.ArActionTriangleDriveTo_deactivate(swigCPtr, this);
  }

  public void setRobot(ArRobot robot) {
    AriaJavaJNI.ArActionTriangleDriveTo_setRobot(swigCPtr, this, ArRobot.getCPtr(robot), robot);
  }

  public ArActionDesired fire(ArActionDesired currentDesired) {
    long cPtr = AriaJavaJNI.ArActionTriangleDriveTo_fire(swigCPtr, this, ArActionDesired.getCPtr(currentDesired), currentDesired);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

  public ArActionDesired getDesired() {
    long cPtr = AriaJavaJNI.ArActionTriangleDriveTo_getDesired(swigCPtr, this);
    return (cPtr == 0) ? null : new ArActionDesired(cPtr, false);
  }

  public final static class State {
    public final static ArActionTriangleDriveTo.State STATE_INACTIVE = new ArActionTriangleDriveTo.State("STATE_INACTIVE");
    public final static ArActionTriangleDriveTo.State STATE_ACQUIRE = new ArActionTriangleDriveTo.State("STATE_ACQUIRE");
    public final static ArActionTriangleDriveTo.State STATE_SEARCHING = new ArActionTriangleDriveTo.State("STATE_SEARCHING");
    public final static ArActionTriangleDriveTo.State STATE_GOTO_APPROACH = new ArActionTriangleDriveTo.State("STATE_GOTO_APPROACH");
    public final static ArActionTriangleDriveTo.State STATE_ALIGN_APPROACH = new ArActionTriangleDriveTo.State("STATE_ALIGN_APPROACH");
    public final static ArActionTriangleDriveTo.State STATE_GOTO_VERTEX = new ArActionTriangleDriveTo.State("STATE_GOTO_VERTEX");
    public final static ArActionTriangleDriveTo.State STATE_GOTO_FINAL = new ArActionTriangleDriveTo.State("STATE_GOTO_FINAL");
    public final static ArActionTriangleDriveTo.State STATE_ALIGN_FINAL = new ArActionTriangleDriveTo.State("STATE_ALIGN_FINAL");
    public final static ArActionTriangleDriveTo.State STATE_SUCCEEDED = new ArActionTriangleDriveTo.State("STATE_SUCCEEDED");
    public final static ArActionTriangleDriveTo.State STATE_FAILED = new ArActionTriangleDriveTo.State("STATE_FAILED");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static State swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + State.class + " with value " + swigValue);
    }

    private State(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private State(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private State(String swigName, State swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static State[] swigValues = { STATE_INACTIVE, STATE_ACQUIRE, STATE_SEARCHING, STATE_GOTO_APPROACH, STATE_ALIGN_APPROACH, STATE_GOTO_VERTEX, STATE_GOTO_FINAL, STATE_ALIGN_FINAL, STATE_SUCCEEDED, STATE_FAILED };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
