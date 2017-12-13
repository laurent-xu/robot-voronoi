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

public class ArActionDesired {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArActionDesired(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArActionDesired obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArActionDesired(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public static double getNO_STRENGTH() {
    return AriaJavaJNI.ArActionDesired_NO_STRENGTH_get();
  }

  public static double getMIN_STRENGTH() {
    return AriaJavaJNI.ArActionDesired_MIN_STRENGTH_get();
  }

  public static double getMAX_STRENGTH() {
    return AriaJavaJNI.ArActionDesired_MAX_STRENGTH_get();
  }

  public ArActionDesired() {
    this(AriaJavaJNI.new_ArActionDesired(), true);
  }

  public void setVel(double vel, double strength) {
    AriaJavaJNI.ArActionDesired_setVel__SWIG_0(swigCPtr, this, vel, strength);
  }

  public void setVel(double vel) {
    AriaJavaJNI.ArActionDesired_setVel__SWIG_1(swigCPtr, this, vel);
  }

  public void setDeltaHeading(double deltaHeading, double strength) {
    AriaJavaJNI.ArActionDesired_setDeltaHeading__SWIG_0(swigCPtr, this, deltaHeading, strength);
  }

  public void setDeltaHeading(double deltaHeading) {
    AriaJavaJNI.ArActionDesired_setDeltaHeading__SWIG_1(swigCPtr, this, deltaHeading);
  }

  public void setHeading(double heading, double strength) {
    AriaJavaJNI.ArActionDesired_setHeading__SWIG_0(swigCPtr, this, heading, strength);
  }

  public void setHeading(double heading) {
    AriaJavaJNI.ArActionDesired_setHeading__SWIG_1(swigCPtr, this, heading);
  }

  public void setRotVel(double rotVel, double strength) {
    AriaJavaJNI.ArActionDesired_setRotVel__SWIG_0(swigCPtr, this, rotVel, strength);
  }

  public void setRotVel(double rotVel) {
    AriaJavaJNI.ArActionDesired_setRotVel__SWIG_1(swigCPtr, this, rotVel);
  }

  public void setMaxVel(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxVel__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxVel(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxVel__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxVel(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxVel__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setMaxNegVel(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxNegVel__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxNegVel(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxNegVel__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxNegVel(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxNegVel__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setTransAccel(double transAccel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setTransAccel__SWIG_0(swigCPtr, this, transAccel, strength, useSlowest);
  }

  public void setTransAccel(double transAccel, double strength) {
    AriaJavaJNI.ArActionDesired_setTransAccel__SWIG_1(swigCPtr, this, transAccel, strength);
  }

  public void setTransAccel(double transAccel) {
    AriaJavaJNI.ArActionDesired_setTransAccel__SWIG_2(swigCPtr, this, transAccel);
  }

  public void setTransDecel(double transDecel, double strength, boolean useFastestDecel) {
    AriaJavaJNI.ArActionDesired_setTransDecel__SWIG_0(swigCPtr, this, transDecel, strength, useFastestDecel);
  }

  public void setTransDecel(double transDecel, double strength) {
    AriaJavaJNI.ArActionDesired_setTransDecel__SWIG_1(swigCPtr, this, transDecel, strength);
  }

  public void setTransDecel(double transDecel) {
    AriaJavaJNI.ArActionDesired_setTransDecel__SWIG_2(swigCPtr, this, transDecel);
  }

  public void setMaxRotVel(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxRotVel__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxRotVel(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxRotVel__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxRotVel(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxRotVel__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setMaxRotVelPos(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelPos__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxRotVelPos(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelPos__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxRotVelPos(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelPos__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setMaxRotVelNeg(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelNeg__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxRotVelNeg(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelNeg__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxRotVelNeg(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxRotVelNeg__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setRotAccel(double rotAccel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setRotAccel__SWIG_0(swigCPtr, this, rotAccel, strength, useSlowest);
  }

  public void setRotAccel(double rotAccel, double strength) {
    AriaJavaJNI.ArActionDesired_setRotAccel__SWIG_1(swigCPtr, this, rotAccel, strength);
  }

  public void setRotAccel(double rotAccel) {
    AriaJavaJNI.ArActionDesired_setRotAccel__SWIG_2(swigCPtr, this, rotAccel);
  }

  public void setRotDecel(double rotDecel, double strength, boolean useFastest) {
    AriaJavaJNI.ArActionDesired_setRotDecel__SWIG_0(swigCPtr, this, rotDecel, strength, useFastest);
  }

  public void setRotDecel(double rotDecel, double strength) {
    AriaJavaJNI.ArActionDesired_setRotDecel__SWIG_1(swigCPtr, this, rotDecel, strength);
  }

  public void setRotDecel(double rotDecel) {
    AriaJavaJNI.ArActionDesired_setRotDecel__SWIG_2(swigCPtr, this, rotDecel);
  }

  public void setLeftLatVel(double latVel, double strength) {
    AriaJavaJNI.ArActionDesired_setLeftLatVel__SWIG_0(swigCPtr, this, latVel, strength);
  }

  public void setLeftLatVel(double latVel) {
    AriaJavaJNI.ArActionDesired_setLeftLatVel__SWIG_1(swigCPtr, this, latVel);
  }

  public void setRightLatVel(double latVel, double strength) {
    AriaJavaJNI.ArActionDesired_setRightLatVel__SWIG_0(swigCPtr, this, latVel, strength);
  }

  public void setRightLatVel(double latVel) {
    AriaJavaJNI.ArActionDesired_setRightLatVel__SWIG_1(swigCPtr, this, latVel);
  }

  public void setMaxLeftLatVel(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxLeftLatVel__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxLeftLatVel(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxLeftLatVel__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxLeftLatVel(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxLeftLatVel__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setMaxRightLatVel(double maxVel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setMaxRightLatVel__SWIG_0(swigCPtr, this, maxVel, strength, useSlowest);
  }

  public void setMaxRightLatVel(double maxVel, double strength) {
    AriaJavaJNI.ArActionDesired_setMaxRightLatVel__SWIG_1(swigCPtr, this, maxVel, strength);
  }

  public void setMaxRightLatVel(double maxVel) {
    AriaJavaJNI.ArActionDesired_setMaxRightLatVel__SWIG_2(swigCPtr, this, maxVel);
  }

  public void setLatAccel(double latAccel, double strength, boolean useSlowest) {
    AriaJavaJNI.ArActionDesired_setLatAccel__SWIG_0(swigCPtr, this, latAccel, strength, useSlowest);
  }

  public void setLatAccel(double latAccel, double strength) {
    AriaJavaJNI.ArActionDesired_setLatAccel__SWIG_1(swigCPtr, this, latAccel, strength);
  }

  public void setLatAccel(double latAccel) {
    AriaJavaJNI.ArActionDesired_setLatAccel__SWIG_2(swigCPtr, this, latAccel);
  }

  public void setLatDecel(double latDecel, double strength, boolean useFastest) {
    AriaJavaJNI.ArActionDesired_setLatDecel__SWIG_0(swigCPtr, this, latDecel, strength, useFastest);
  }

  public void setLatDecel(double latDecel, double strength) {
    AriaJavaJNI.ArActionDesired_setLatDecel__SWIG_1(swigCPtr, this, latDecel, strength);
  }

  public void setLatDecel(double latDecel) {
    AriaJavaJNI.ArActionDesired_setLatDecel__SWIG_2(swigCPtr, this, latDecel);
  }

  public void reset() {
    AriaJavaJNI.ArActionDesired_reset(swigCPtr, this);
  }

  public double getVel() {
    return AriaJavaJNI.ArActionDesired_getVel(swigCPtr, this);
  }

  public double getVelStrength() {
    return AriaJavaJNI.ArActionDesired_getVelStrength(swigCPtr, this);
  }

  public double getHeading() {
    return AriaJavaJNI.ArActionDesired_getHeading(swigCPtr, this);
  }

  public double getHeadingStrength() {
    return AriaJavaJNI.ArActionDesired_getHeadingStrength(swigCPtr, this);
  }

  public double getDeltaHeading() {
    return AriaJavaJNI.ArActionDesired_getDeltaHeading(swigCPtr, this);
  }

  public double getDeltaHeadingStrength() {
    return AriaJavaJNI.ArActionDesired_getDeltaHeadingStrength(swigCPtr, this);
  }

  public double getRotVel() {
    return AriaJavaJNI.ArActionDesired_getRotVel(swigCPtr, this);
  }

  public double getRotVelStrength() {
    return AriaJavaJNI.ArActionDesired_getRotVelStrength(swigCPtr, this);
  }

  public double getMaxVel() {
    return AriaJavaJNI.ArActionDesired_getMaxVel(swigCPtr, this);
  }

  public double getMaxVelStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxVelStrength(swigCPtr, this);
  }

  public double getMaxVelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxVelSlowestUsed(swigCPtr, this);
  }

  public double getMaxNegVel() {
    return AriaJavaJNI.ArActionDesired_getMaxNegVel(swigCPtr, this);
  }

  public double getMaxNegVelStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxNegVelStrength(swigCPtr, this);
  }

  public double getMaxNegVelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxNegVelSlowestUsed(swigCPtr, this);
  }

  public double getTransAccel() {
    return AriaJavaJNI.ArActionDesired_getTransAccel(swigCPtr, this);
  }

  public double getTransAccelStrength() {
    return AriaJavaJNI.ArActionDesired_getTransAccelStrength(swigCPtr, this);
  }

  public double getTransAccelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getTransAccelSlowestUsed(swigCPtr, this);
  }

  public double getTransDecel() {
    return AriaJavaJNI.ArActionDesired_getTransDecel(swigCPtr, this);
  }

  public double getTransDecelStrength() {
    return AriaJavaJNI.ArActionDesired_getTransDecelStrength(swigCPtr, this);
  }

  public double getTransDecelFastestUsed() {
    return AriaJavaJNI.ArActionDesired_getTransDecelFastestUsed(swigCPtr, this);
  }

  public double getMaxRotVel() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVel(swigCPtr, this);
  }

  public double getMaxRotVelStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelStrength(swigCPtr, this);
  }

  public double getMaxRotVelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelSlowestUsed(swigCPtr, this);
  }

  public double getMaxRotVelPos() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelPos(swigCPtr, this);
  }

  public double getMaxRotVelPosStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelPosStrength(swigCPtr, this);
  }

  public double getMaxRotVelPosSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelPosSlowestUsed(swigCPtr, this);
  }

  public double getMaxRotVelNeg() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelNeg(swigCPtr, this);
  }

  public double getMaxRotVelNegStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelNegStrength(swigCPtr, this);
  }

  public double getMaxRotVelNegSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxRotVelNegSlowestUsed(swigCPtr, this);
  }

  public double getRotAccel() {
    return AriaJavaJNI.ArActionDesired_getRotAccel(swigCPtr, this);
  }

  public double getRotAccelStrength() {
    return AriaJavaJNI.ArActionDesired_getRotAccelStrength(swigCPtr, this);
  }

  public double getRotAccelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getRotAccelSlowestUsed(swigCPtr, this);
  }

  public double getRotDecel() {
    return AriaJavaJNI.ArActionDesired_getRotDecel(swigCPtr, this);
  }

  public double getRotDecelStrength() {
    return AriaJavaJNI.ArActionDesired_getRotDecelStrength(swigCPtr, this);
  }

  public double getRotDecelFastestUsed() {
    return AriaJavaJNI.ArActionDesired_getRotDecelFastestUsed(swigCPtr, this);
  }

  public double getLatVel() {
    return AriaJavaJNI.ArActionDesired_getLatVel(swigCPtr, this);
  }

  public double getLatVelStrength() {
    return AriaJavaJNI.ArActionDesired_getLatVelStrength(swigCPtr, this);
  }

  public double getMaxLeftLatVel() {
    return AriaJavaJNI.ArActionDesired_getMaxLeftLatVel(swigCPtr, this);
  }

  public double getMaxLeftLatVelStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxLeftLatVelStrength(swigCPtr, this);
  }

  public double getMaxLeftLatVelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxLeftLatVelSlowestUsed(swigCPtr, this);
  }

  public double getMaxRightLatVel() {
    return AriaJavaJNI.ArActionDesired_getMaxRightLatVel(swigCPtr, this);
  }

  public double getMaxRightLatVelStrength() {
    return AriaJavaJNI.ArActionDesired_getMaxRightLatVelStrength(swigCPtr, this);
  }

  public double getMaxRightLatVelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getMaxRightLatVelSlowestUsed(swigCPtr, this);
  }

  public double getLatAccel() {
    return AriaJavaJNI.ArActionDesired_getLatAccel(swigCPtr, this);
  }

  public double getLatAccelStrength() {
    return AriaJavaJNI.ArActionDesired_getLatAccelStrength(swigCPtr, this);
  }

  public double getLatAccelSlowestUsed() {
    return AriaJavaJNI.ArActionDesired_getLatAccelSlowestUsed(swigCPtr, this);
  }

  public double getLatDecel() {
    return AriaJavaJNI.ArActionDesired_getLatDecel(swigCPtr, this);
  }

  public double getLatDecelStrength() {
    return AriaJavaJNI.ArActionDesired_getLatDecelStrength(swigCPtr, this);
  }

  public double getLatDecelFastestUsed() {
    return AriaJavaJNI.ArActionDesired_getLatDecelFastestUsed(swigCPtr, this);
  }

  public void merge(ArActionDesired actDesired) {
    AriaJavaJNI.ArActionDesired_merge(swigCPtr, this, ArActionDesired.getCPtr(actDesired), actDesired);
  }

  public void startAverage() {
    AriaJavaJNI.ArActionDesired_startAverage(swigCPtr, this);
  }

  public void addAverage(ArActionDesired actDesired) {
    AriaJavaJNI.ArActionDesired_addAverage(swigCPtr, this, ArActionDesired.getCPtr(actDesired), actDesired);
  }

  public void endAverage() {
    AriaJavaJNI.ArActionDesired_endAverage(swigCPtr, this);
  }

  public void accountForRobotHeading(double robotHeading) {
    AriaJavaJNI.ArActionDesired_accountForRobotHeading(swigCPtr, this, robotHeading);
  }

  public void log() {
    AriaJavaJNI.ArActionDesired_log(swigCPtr, this);
  }

  public boolean isAnythingDesired() {
    return AriaJavaJNI.ArActionDesired_isAnythingDesired(swigCPtr, this);
  }

  public void sanityCheck(String actionName) {
    AriaJavaJNI.ArActionDesired_sanityCheck(swigCPtr, this, actionName);
  }

}