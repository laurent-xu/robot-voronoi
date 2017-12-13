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

public class ArKeyHandler {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArKeyHandler(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArKeyHandler obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArKeyHandler(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public ArKeyHandler(boolean blocking, boolean addAriaExitCB, SWIGTYPE_p_FILE stream, boolean takeKeysInConstructor) {
    this(AriaJavaJNI.new_ArKeyHandler__SWIG_0(blocking, addAriaExitCB, SWIGTYPE_p_FILE.getCPtr(stream), takeKeysInConstructor), true);
  }

  public ArKeyHandler(boolean blocking, boolean addAriaExitCB, SWIGTYPE_p_FILE stream) {
    this(AriaJavaJNI.new_ArKeyHandler__SWIG_1(blocking, addAriaExitCB, SWIGTYPE_p_FILE.getCPtr(stream)), true);
  }

  public ArKeyHandler(boolean blocking, boolean addAriaExitCB) {
    this(AriaJavaJNI.new_ArKeyHandler__SWIG_2(blocking, addAriaExitCB), true);
  }

  public ArKeyHandler(boolean blocking) {
    this(AriaJavaJNI.new_ArKeyHandler__SWIG_3(blocking), true);
  }

  public ArKeyHandler() {
    this(AriaJavaJNI.new_ArKeyHandler__SWIG_4(), true);
  }

  public boolean addKeyHandler(int keyToHandle, ArFunctor functor) {
    return AriaJavaJNI.ArKeyHandler_addKeyHandler(swigCPtr, this, keyToHandle, ArFunctor.getCPtr(functor), functor);
  }

  public boolean remKeyHandler(int keyToHandler) {
    return AriaJavaJNI.ArKeyHandler_remKeyHandler__SWIG_0(swigCPtr, this, keyToHandler);
  }

  public boolean remKeyHandler(ArFunctor functor) {
    return AriaJavaJNI.ArKeyHandler_remKeyHandler__SWIG_1(swigCPtr, this, ArFunctor.getCPtr(functor), functor);
  }

  public void takeKeys(boolean blocking) {
    AriaJavaJNI.ArKeyHandler_takeKeys__SWIG_0(swigCPtr, this, blocking);
  }

  public void takeKeys() {
    AriaJavaJNI.ArKeyHandler_takeKeys__SWIG_1(swigCPtr, this);
  }

  public void restore() {
    AriaJavaJNI.ArKeyHandler_restore(swigCPtr, this);
  }

  public void checkKeys() {
    AriaJavaJNI.ArKeyHandler_checkKeys(swigCPtr, this);
  }

  public int getKey() {
    return AriaJavaJNI.ArKeyHandler_getKey(swigCPtr, this);
  }

  public final static class KEY {
    public final static ArKeyHandler.KEY UP = new ArKeyHandler.KEY("UP", AriaJavaJNI.ArKeyHandler_UP_get());
    public final static ArKeyHandler.KEY DOWN = new ArKeyHandler.KEY("DOWN");
    public final static ArKeyHandler.KEY LEFT = new ArKeyHandler.KEY("LEFT");
    public final static ArKeyHandler.KEY RIGHT = new ArKeyHandler.KEY("RIGHT");
    public final static ArKeyHandler.KEY ESCAPE = new ArKeyHandler.KEY("ESCAPE");
    public final static ArKeyHandler.KEY SPACE = new ArKeyHandler.KEY("SPACE");
    public final static ArKeyHandler.KEY TAB = new ArKeyHandler.KEY("TAB");
    public final static ArKeyHandler.KEY ENTER = new ArKeyHandler.KEY("ENTER");
    public final static ArKeyHandler.KEY BACKSPACE = new ArKeyHandler.KEY("BACKSPACE");
    public final static ArKeyHandler.KEY _StartFKeys = new ArKeyHandler.KEY("_StartFKeys");
    public final static ArKeyHandler.KEY F1 = new ArKeyHandler.KEY("F1");
    public final static ArKeyHandler.KEY F2 = new ArKeyHandler.KEY("F2");
    public final static ArKeyHandler.KEY F3 = new ArKeyHandler.KEY("F3");
    public final static ArKeyHandler.KEY F4 = new ArKeyHandler.KEY("F4");
    public final static ArKeyHandler.KEY F5 = new ArKeyHandler.KEY("F5");
    public final static ArKeyHandler.KEY F6 = new ArKeyHandler.KEY("F6");
    public final static ArKeyHandler.KEY F7 = new ArKeyHandler.KEY("F7");
    public final static ArKeyHandler.KEY F8 = new ArKeyHandler.KEY("F8");
    public final static ArKeyHandler.KEY F9 = new ArKeyHandler.KEY("F9");
    public final static ArKeyHandler.KEY F10 = new ArKeyHandler.KEY("F10");
    public final static ArKeyHandler.KEY F11 = new ArKeyHandler.KEY("F11");
    public final static ArKeyHandler.KEY F12 = new ArKeyHandler.KEY("F12");
    public final static ArKeyHandler.KEY _EndFKeys = new ArKeyHandler.KEY("_EndFKeys");
    public final static ArKeyHandler.KEY PAGEUP = new ArKeyHandler.KEY("PAGEUP");
    public final static ArKeyHandler.KEY PAGEDOWN = new ArKeyHandler.KEY("PAGEDOWN");
    public final static ArKeyHandler.KEY HOME = new ArKeyHandler.KEY("HOME");
    public final static ArKeyHandler.KEY END = new ArKeyHandler.KEY("END");
    public final static ArKeyHandler.KEY INSERT = new ArKeyHandler.KEY("INSERT");
    public final static ArKeyHandler.KEY DEL = new ArKeyHandler.KEY("DEL");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static KEY swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + KEY.class + " with value " + swigValue);
    }

    private KEY(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private KEY(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private KEY(String swigName, KEY swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static KEY[] swigValues = { UP, DOWN, LEFT, RIGHT, ESCAPE, SPACE, TAB, ENTER, BACKSPACE, _StartFKeys, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, _EndFKeys, PAGEUP, PAGEDOWN, HOME, END, INSERT, DEL };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
