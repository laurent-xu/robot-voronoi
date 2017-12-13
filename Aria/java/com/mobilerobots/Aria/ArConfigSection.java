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

public class ArConfigSection {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArConfigSection(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArConfigSection obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArConfigSection(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArConfigSection(String name, String comment, boolean isQuiet, String categoryName) {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_0(name, comment, isQuiet, categoryName), true);
  }

  public ArConfigSection(String name, String comment, boolean isQuiet) {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_1(name, comment, isQuiet), true);
  }

  public ArConfigSection(String name, String comment) {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_2(name, comment), true);
  }

  public ArConfigSection(String name) {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_3(name), true);
  }

  public ArConfigSection() {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_4(), true);
  }

  public ArConfigSection(ArConfigSection section) {
    this(AriaJavaJNI.new_ArConfigSection__SWIG_5(ArConfigSection.getCPtr(section), section), true);
  }

  public void copyAndDetach(ArConfigSection section) {
    AriaJavaJNI.ArConfigSection_copyAndDetach(swigCPtr, this, ArConfigSection.getCPtr(section), section);
  }

  public String getName() {
    return AriaJavaJNI.ArConfigSection_getName(swigCPtr, this);
  }

  public String getComment() {
    return AriaJavaJNI.ArConfigSection_getComment(swigCPtr, this);
  }

  public String getCategoryName() {
    return AriaJavaJNI.ArConfigSection_getCategoryName(swigCPtr, this);
  }

  public String getFlags() {
    return AriaJavaJNI.ArConfigSection_getFlags(swigCPtr, this);
  }

  public boolean hasFlag(String flag) {
    return AriaJavaJNI.ArConfigSection_hasFlag(swigCPtr, this, flag);
  }

  public SWIGTYPE_p_std__listT_ArConfigArg_t getParams() {
    long cPtr = AriaJavaJNI.ArConfigSection_getParams(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_std__listT_ArConfigArg_t(cPtr, false);
  }

  public void setName(String name) {
    AriaJavaJNI.ArConfigSection_setName(swigCPtr, this, name);
  }

  public void setComment(String comment) {
    AriaJavaJNI.ArConfigSection_setComment(swigCPtr, this, comment);
  }

  public boolean addFlags(String flags, boolean isQuiet) {
    return AriaJavaJNI.ArConfigSection_addFlags__SWIG_0(swigCPtr, this, flags, isQuiet);
  }

  public boolean addFlags(String flags) {
    return AriaJavaJNI.ArConfigSection_addFlags__SWIG_1(swigCPtr, this, flags);
  }

  public boolean remFlag(String dataFlag) {
    return AriaJavaJNI.ArConfigSection_remFlag(swigCPtr, this, dataFlag);
  }

  public ArConfigArg findParam(String paramName, boolean isAllowStringHolders) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_0(swigCPtr, this, paramName, isAllowStringHolders);
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public ArConfigArg findParam(String paramName) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_1(swigCPtr, this, paramName);
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public ArConfigArg findParam(SWIGTYPE_p_std__listT_std__string_t paramNamePath, boolean isAllowHolders) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_2(swigCPtr, this, SWIGTYPE_p_std__listT_std__string_t.getCPtr(paramNamePath), isAllowHolders);
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public ArConfigArg findParam(SWIGTYPE_p_std__listT_std__string_t paramNamePath) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_3(swigCPtr, this, SWIGTYPE_p_std__listT_std__string_t.getCPtr(paramNamePath));
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public ArConfigArg findParam(SWIGTYPE_p_p_char paramNamePath, int pathLength, boolean isAllowHolders) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_4(swigCPtr, this, SWIGTYPE_p_p_char.getCPtr(paramNamePath), pathLength, isAllowHolders);
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public ArConfigArg findParam(SWIGTYPE_p_p_char paramNamePath, int pathLength) {
    long cPtr = AriaJavaJNI.ArConfigSection_findParam__SWIG_5(swigCPtr, this, SWIGTYPE_p_p_char.getCPtr(paramNamePath), pathLength);
    return (cPtr == 0) ? null : new ArConfigArg(cPtr, false);
  }

  public boolean containsParamsOfPriority(ArPriority.Priority highestPriority, ArPriority.Priority lowestPriority) {
    return AriaJavaJNI.ArConfigSection_containsParamsOfPriority(swigCPtr, this, highestPriority.swigValue(), lowestPriority.swigValue());
  }

  public boolean remStringHolder(String paramName) {
    return AriaJavaJNI.ArConfigSection_remStringHolder(swigCPtr, this, paramName);
  }

  public void setQuiet(boolean isQuiet) {
    AriaJavaJNI.ArConfigSection_setQuiet(swigCPtr, this, isQuiet);
  }

}
