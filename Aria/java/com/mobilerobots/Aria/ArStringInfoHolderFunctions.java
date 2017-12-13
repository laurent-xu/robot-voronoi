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

public class ArStringInfoHolderFunctions {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  public ArStringInfoHolderFunctions(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArStringInfoHolderFunctions obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArStringInfoHolderFunctions(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public static void intWrapper(String buffer, int bufferLen, ArRetFunctor_Int functor, String format, int navalue) {
    AriaJavaJNI.ArStringInfoHolderFunctions_intWrapper(buffer, bufferLen, ArRetFunctor_Int.getCPtr(functor), functor, format, navalue);
  }

  public static void doubleWrapper(String buffer, int bufferLen, ArRetFunctor_Double functor, String format) {
    AriaJavaJNI.ArStringInfoHolderFunctions_doubleWrapper(buffer, bufferLen, ArRetFunctor_Double.getCPtr(functor), functor, format);
  }

  public static void boolWrapper(String buffer, int bufferLen, ArRetFunctor_Bool functor, String format) {
    AriaJavaJNI.ArStringInfoHolderFunctions_boolWrapper(buffer, bufferLen, ArRetFunctor_Bool.getCPtr(functor), functor, format);
  }

  public static void stringWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_char_const_p_t functor, String format) {
    AriaJavaJNI.ArStringInfoHolderFunctions_stringWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_char_const_p_t.getCPtr(functor), format);
  }

  public static void cppStringWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_std__string_t functor) {
    AriaJavaJNI.ArStringInfoHolderFunctions_cppStringWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_std__string_t.getCPtr(functor));
  }

  public static void unsignedLongWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_unsigned_long_t functor, String format, long navalue) {
    AriaJavaJNI.ArStringInfoHolderFunctions_unsignedLongWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_unsigned_long_t.getCPtr(functor), format, navalue);
  }

  public static void longWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_long_t functor, String format, int navalue) {
    AriaJavaJNI.ArStringInfoHolderFunctions_longWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_long_t.getCPtr(functor), format, navalue);
  }

  public static void arTimeWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_ArTime_t functor, String format) {
    AriaJavaJNI.ArStringInfoHolderFunctions_arTimeWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_ArTime_t.getCPtr(functor), format);
  }

  public static void floatWrapper(String buffer, int bufferLen, SWIGTYPE_p_ArRetFunctorT_float_t functor, String format) {
    AriaJavaJNI.ArStringInfoHolderFunctions_floatWrapper(buffer, bufferLen, SWIGTYPE_p_ArRetFunctorT_float_t.getCPtr(functor), format);
  }

  public ArStringInfoHolderFunctions() {
    this(AriaJavaJNI.new_ArStringInfoHolderFunctions(), true);
  }

}
