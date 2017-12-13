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
public class ArNetPacket extends ArBasePacket {
  private transient long swigCPtr;

  public ArNetPacket(long cPtr, boolean cMemoryOwn) {
    super(ArNetworkingJavaJNI.ArNetPacket_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArNetPacket obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        ArNetworkingJavaJNI.delete_ArNetPacket(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public ArNetPacket(int bufferSize) {
    this(ArNetworkingJavaJNI.new_ArNetPacket__SWIG_0(bufferSize), true);
  }

  public ArNetPacket() {
    this(ArNetworkingJavaJNI.new_ArNetPacket__SWIG_1(), true);
  }

  public ArNetPacket(ArNetPacket other) {
    this(ArNetworkingJavaJNI.new_ArNetPacket__SWIG_2(ArNetPacket.getCPtr(other), other), true);
  }

  public void setCommand(int command) {
    ArNetworkingJavaJNI.ArNetPacket_setCommand(swigCPtr, this, command);
  }

  public int getCommand() {
    return ArNetworkingJavaJNI.ArNetPacket_getCommand(swigCPtr, this);
  }

  public void doubleToBuf(double val) {
    ArNetworkingJavaJNI.ArNetPacket_doubleToBuf(swigCPtr, this, val);
  }

  public double bufToDouble() {
    return ArNetworkingJavaJNI.ArNetPacket_bufToDouble(swigCPtr, this);
  }

  public void empty() {
    ArNetworkingJavaJNI.ArNetPacket_empty(swigCPtr, this);
  }

  public void finalizePacket() {
    ArNetworkingJavaJNI.ArNetPacket_finalizePacket(swigCPtr, this);
  }

  public void resetRead() {
    ArNetworkingJavaJNI.ArNetPacket_resetRead(swigCPtr, this);
  }

  public void duplicatePacket(ArNetPacket packet) {
    ArNetworkingJavaJNI.ArNetPacket_duplicatePacket(swigCPtr, this, ArNetPacket.getCPtr(packet), packet);
  }

  public boolean verifyCheckSum() {
    return ArNetworkingJavaJNI.ArNetPacket_verifyCheckSum(swigCPtr, this);
  }

  public short calcCheckSum() {
    return ArNetworkingJavaJNI.ArNetPacket_calcCheckSum(swigCPtr, this);
  }

  public boolean getAddedFooter() {
    return ArNetworkingJavaJNI.ArNetPacket_getAddedFooter(swigCPtr, this);
  }

  public void setAddedFooter(boolean addedFooter) {
    ArNetworkingJavaJNI.ArNetPacket_setAddedFooter(swigCPtr, this, addedFooter);
  }

  public ArNetPacket.PacketSource getPacketSource() {
    return ArNetPacket.PacketSource.swigToEnum(ArNetworkingJavaJNI.ArNetPacket_getPacketSource(swigCPtr, this));
  }

  public void setPacketSource(ArNetPacket.PacketSource source) {
    ArNetworkingJavaJNI.ArNetPacket_setPacketSource(swigCPtr, this, source.swigValue());
  }

  public void setArbitraryString(String string) {
    ArNetworkingJavaJNI.ArNetPacket_setArbitraryString(swigCPtr, this, string);
  }

  public String getArbitraryString() {
    return ArNetworkingJavaJNI.ArNetPacket_getArbitraryString(swigCPtr, this);
  }

  public final static int SIZE_OF_LENGTH = ArNetworkingJavaJNI.ArNetPacket_SIZE_OF_LENGTH_get();
  public final static int MAX_LENGTH = ArNetworkingJavaJNI.ArNetPacket_MAX_LENGTH_get();
  public final static int HEADER_LENGTH = ArNetworkingJavaJNI.ArNetPacket_HEADER_LENGTH_get();
  public final static int FOOTER_LENGTH = ArNetworkingJavaJNI.ArNetPacket_FOOTER_LENGTH_get();
  public final static int MAX_DATA_LENGTH = ArNetworkingJavaJNI.ArNetPacket_MAX_DATA_LENGTH_get();

  public final static class PacketSource {
    public final static ArNetPacket.PacketSource TCP = new ArNetPacket.PacketSource("TCP");
    public final static ArNetPacket.PacketSource UDP = new ArNetPacket.PacketSource("UDP");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static PacketSource swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + PacketSource.class + " with value " + swigValue);
    }

    private PacketSource(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private PacketSource(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private PacketSource(String swigName, PacketSource swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static PacketSource[] swigValues = { TCP, UDP };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
