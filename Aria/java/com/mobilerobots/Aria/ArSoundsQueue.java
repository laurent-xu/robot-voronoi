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

public class ArSoundsQueue extends ArASyncTask {
  private transient long swigCPtr;

  public ArSoundsQueue(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.ArSoundsQueue_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr(ArSoundsQueue obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        AriaJavaJNI.delete_ArSoundsQueue(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  static public class Item {
    private transient long swigCPtr;
    protected transient boolean swigCMemOwn;
  
    public Item(long cPtr, boolean cMemoryOwn) {
      swigCMemOwn = cMemoryOwn;
      swigCPtr = cPtr;
    }
  
    public static long getCPtr(Item obj) {
      return (obj == null) ? 0 : obj.swigCPtr;
    }
  
    protected void finalize() {
      delete();
    }
  
    public synchronized void delete() {
      if (swigCPtr != 0) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          AriaJavaJNI.delete_ArSoundsQueue_Item(swigCPtr);
        }
        swigCPtr = 0;
      }
    }
  
    public void setData(String value) {
      AriaJavaJNI.ArSoundsQueue_Item_data_set(swigCPtr, this, value);
    }
  
    public String getData() {
      return AriaJavaJNI.ArSoundsQueue_Item_data_get(swigCPtr, this);
    }
  
    public void setType(ArSoundsQueue.ItemType value) {
      AriaJavaJNI.ArSoundsQueue_Item_type_set(swigCPtr, this, value.swigValue());
    }
  
    public ArSoundsQueue.ItemType getType() {
      return ArSoundsQueue.ItemType.swigToEnum(AriaJavaJNI.ArSoundsQueue_Item_type_get(swigCPtr, this));
    }
  
    public void setParams(String value) {
      AriaJavaJNI.ArSoundsQueue_Item_params_set(swigCPtr, this, value);
    }
  
    public String getParams() {
      return AriaJavaJNI.ArSoundsQueue_Item_params_get(swigCPtr, this);
    }
  
    public void setPriority(int value) {
      AriaJavaJNI.ArSoundsQueue_Item_priority_set(swigCPtr, this, value);
    }
  
    public int getPriority() {
      return AriaJavaJNI.ArSoundsQueue_Item_priority_get(swigCPtr, this);
    }
  
    public void setInterruptCallbacks(ArFunctorPtrList value) {
      AriaJavaJNI.ArSoundsQueue_Item_interruptCallbacks_set(swigCPtr, this, ArFunctorPtrList.getCPtr(value), value);
    }
  
    public ArFunctorPtrList getInterruptCallbacks() {
      long cPtr = AriaJavaJNI.ArSoundsQueue_Item_interruptCallbacks_get(swigCPtr, this);
      return (cPtr == 0) ? null : new ArFunctorPtrList(cPtr, false);
    }
  
    public void setPlayCallbacks(SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t value) {
      AriaJavaJNI.ArSoundsQueue_Item_playCallbacks_set(swigCPtr, this, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t.getCPtr(value));
    }
  
    public SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t getPlayCallbacks() {
      long cPtr = AriaJavaJNI.ArSoundsQueue_Item_playCallbacks_get(swigCPtr, this);
      return (cPtr == 0) ? null : new SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t(cPtr, false);
    }
  
    public void setDoneCallbacks(ArFunctorPtrList value) {
      AriaJavaJNI.ArSoundsQueue_Item_doneCallbacks_set(swigCPtr, this, ArFunctorPtrList.getCPtr(value), value);
    }
  
    public ArFunctorPtrList getDoneCallbacks() {
      long cPtr = AriaJavaJNI.ArSoundsQueue_Item_doneCallbacks_get(swigCPtr, this);
      return (cPtr == 0) ? null : new ArFunctorPtrList(cPtr, false);
    }
  
    public void setPlaybackConditionCallbacks(SWIGTYPE_p_std__listT_ArRetFunctorT_bool_t_p_t value) {
      AriaJavaJNI.ArSoundsQueue_Item_playbackConditionCallbacks_set(swigCPtr, this, SWIGTYPE_p_std__listT_ArRetFunctorT_bool_t_p_t.getCPtr(value));
    }
  
    public SWIGTYPE_p_std__listT_ArRetFunctorT_bool_t_p_t getPlaybackConditionCallbacks() {
      long cPtr = AriaJavaJNI.ArSoundsQueue_Item_playbackConditionCallbacks_get(swigCPtr, this);
      return (cPtr == 0) ? null : new SWIGTYPE_p_std__listT_ArRetFunctorT_bool_t_p_t(cPtr, false);
    }
  
    public Item() {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_0(), true);
    }
  
    public Item(String _data, ArSoundsQueue.ItemType _type, String _params, int priority) {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_1(_data, _type.swigValue(), _params, priority), true);
    }
  
    public Item(String _data, ArSoundsQueue.ItemType _type, String _params) {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_2(_data, _type.swigValue(), _params), true);
    }
  
    public Item(String _data, ArSoundsQueue.ItemType _type) {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_3(_data, _type.swigValue()), true);
    }
  
    public Item(String _data, ArSoundsQueue.ItemType _type, String _params, int priority, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t callbacks) {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_4(_data, _type.swigValue(), _params, priority, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t.getCPtr(callbacks)), true);
    }
  
    public Item(ArSoundsQueue.Item toCopy) {
      this(AriaJavaJNI.new_ArSoundsQueue_Item__SWIG_5(ArSoundsQueue.Item.getCPtr(toCopy), toCopy), true);
    }
  
    public void play() {
      AriaJavaJNI.ArSoundsQueue_Item_play(swigCPtr, this);
    }
  
    public void interrupt() {
      AriaJavaJNI.ArSoundsQueue_Item_interrupt(swigCPtr, this);
    }
  
    public void done() {
      AriaJavaJNI.ArSoundsQueue_Item_done(swigCPtr, this);
    }
  
  }

  public ArSoundsQueue() {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_0(), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t speakCB, ArFunctor interruptSpeechCB, ArRetFunctor_Bool playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t playFileCB, ArFunctor interruptFileCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_1(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(speakCB), ArFunctor.getCPtr(interruptSpeechCB), interruptSpeechCB, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(playFileCB), ArFunctor.getCPtr(interruptFileCB), interruptFileCB), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t speakCB, ArFunctor interruptSpeechCB, ArRetFunctor_Bool playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t playFileCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_2(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(speakCB), ArFunctor.getCPtr(interruptSpeechCB), interruptSpeechCB, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(playFileCB)), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t speakCB, ArFunctor interruptSpeechCB, ArRetFunctor_Bool playInitCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_3(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(speakCB), ArFunctor.getCPtr(interruptSpeechCB), interruptSpeechCB, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t speakCB, ArFunctor interruptSpeechCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_4(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(speakCB), ArFunctor.getCPtr(interruptSpeechCB), interruptSpeechCB), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t speakCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_5(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(speakCB)), true);
  }

  public ArSoundsQueue(ArRetFunctor_Bool speakInitCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_6(ArRetFunctor_Bool.getCPtr(speakInitCB), speakInitCB), true);
  }

  public ArSoundsQueue(ArSpeechSynth speechSynthesizer, ArRetFunctor_Bool playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t playFileCB, ArFunctor interruptFileCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_7(ArSpeechSynth.getCPtr(speechSynthesizer), speechSynthesizer, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(playFileCB), ArFunctor.getCPtr(interruptFileCB), interruptFileCB), true);
  }

  public ArSoundsQueue(ArSpeechSynth speechSynthesizer, ArRetFunctor_Bool playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t playFileCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_8(ArSpeechSynth.getCPtr(speechSynthesizer), speechSynthesizer, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(playFileCB)), true);
  }

  public ArSoundsQueue(ArSpeechSynth speechSynthesizer, ArRetFunctor_Bool playInitCB) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_9(ArSpeechSynth.getCPtr(speechSynthesizer), speechSynthesizer, ArRetFunctor_Bool.getCPtr(playInitCB), playInitCB), true);
  }

  public ArSoundsQueue(ArSpeechSynth speechSynthesizer) {
    this(AriaJavaJNI.new_ArSoundsQueue__SWIG_10(ArSpeechSynth.getCPtr(speechSynthesizer), speechSynthesizer), true);
  }

  public void addInitCallback(ArRetFunctor_Bool cb) {
    AriaJavaJNI.ArSoundsQueue_addInitCallback(swigCPtr, this, ArRetFunctor_Bool.getCPtr(cb), cb);
  }

  public void setSpeakInitCallback(ArRetFunctor_Bool cb) {
    AriaJavaJNI.ArSoundsQueue_setSpeakInitCallback(swigCPtr, this, ArRetFunctor_Bool.getCPtr(cb), cb);
  }

  public void addItem(ArSoundsQueue.Item item) {
    AriaJavaJNI.ArSoundsQueue_addItem__SWIG_0(swigCPtr, this, ArSoundsQueue.Item.getCPtr(item), item);
  }

  public void addItem(ArSoundsQueue.ItemType type, String data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t callbacks, int priority, String params) {
    AriaJavaJNI.ArSoundsQueue_addItem__SWIG_1(swigCPtr, this, type.swigValue(), data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t.getCPtr(callbacks), priority, params);
  }

  public void addItem(ArSoundsQueue.ItemType type, String data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t callbacks, int priority) {
    AriaJavaJNI.ArSoundsQueue_addItem__SWIG_2(swigCPtr, this, type.swigValue(), data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t.getCPtr(callbacks), priority);
  }

  public void addItem(ArSoundsQueue.ItemType type, String data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t callbacks) {
    AriaJavaJNI.ArSoundsQueue_addItem__SWIG_3(swigCPtr, this, type.swigValue(), data, SWIGTYPE_p_std__listT_ArRetFunctor2T_bool_char_const_p_char_const_p_t_p_t.getCPtr(callbacks));
  }

  public boolean isInitialized() {
    return AriaJavaJNI.ArSoundsQueue_isInitialized(swigCPtr, this);
  }

  public boolean isSpeakingOrPlaying() {
    return AriaJavaJNI.ArSoundsQueue_isSpeakingOrPlaying(swigCPtr, this);
  }

  public boolean isPlaying() {
    return AriaJavaJNI.ArSoundsQueue_isPlaying(swigCPtr, this);
  }

  public boolean isSpeaking() {
    return AriaJavaJNI.ArSoundsQueue_isSpeaking(swigCPtr, this);
  }

  public void run() {
    AriaJavaJNI.ArSoundsQueue_run(swigCPtr, this);
  }

  public void runAsync() {
    AriaJavaJNI.ArSoundsQueue_runAsync(swigCPtr, this);
  }

  public void pause() {
    AriaJavaJNI.ArSoundsQueue_pause(swigCPtr, this);
  }

  public void resume() {
    AriaJavaJNI.ArSoundsQueue_resume(swigCPtr, this);
  }

  public boolean isPaused() {
    return AriaJavaJNI.ArSoundsQueue_isPaused(swigCPtr, this);
  }

  public void interrupt() {
    AriaJavaJNI.ArSoundsQueue_interrupt(swigCPtr, this);
  }

  public void clearQueue() {
    AriaJavaJNI.ArSoundsQueue_clearQueue(swigCPtr, this);
  }

  public void stop() {
    AriaJavaJNI.ArSoundsQueue_stop(swigCPtr, this);
  }

  public ArFunctor getPauseCallback() {
    long cPtr = AriaJavaJNI.ArSoundsQueue_getPauseCallback(swigCPtr, this);
    return (cPtr == 0) ? null : new ArFunctor(cPtr, false);
  }

  public ArFunctor getResumeCallback() {
    long cPtr = AriaJavaJNI.ArSoundsQueue_getResumeCallback(swigCPtr, this);
    return (cPtr == 0) ? null : new ArFunctor(cPtr, false);
  }

  public long getCurrentQueueSize() {
    return AriaJavaJNI.ArSoundsQueue_getCurrentQueueSize(swigCPtr, this);
  }

  public void addSoundStartedCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_addSoundStartedCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void remSoundStartedCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_remSoundStartedCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void addSoundFinishedCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_addSoundFinishedCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void remSoundFinishedCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_remSoundFinishedCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void addSoundItemStartedCallback(SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t f) {
    AriaJavaJNI.ArSoundsQueue_addSoundItemStartedCallback(swigCPtr, this, SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t.getCPtr(f));
  }

  public void remSoundItemStartedCallback(SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t f) {
    AriaJavaJNI.ArSoundsQueue_remSoundItemStartedCallback(swigCPtr, this, SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t.getCPtr(f));
  }

  public void addSoundItemFinishedCallback(SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t f) {
    AriaJavaJNI.ArSoundsQueue_addSoundItemFinishedCallback(swigCPtr, this, SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t.getCPtr(f));
  }

  public void remSoundItemFinishedCallback(SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t f) {
    AriaJavaJNI.ArSoundsQueue_remSoundItemFinishedCallback(swigCPtr, this, SWIGTYPE_p_ArFunctor1T_ArSoundsQueue__Item_t.getCPtr(f));
  }

  public void addQueueNonemptyCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_addQueueNonemptyCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void remQueueNonemptyCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_remQueueNonemptyCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void addQueueEmptyCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_addQueueEmptyCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public void remQueueEmptyCallback(ArFunctor f) {
    AriaJavaJNI.ArSoundsQueue_remQueueEmptyCallback(swigCPtr, this, ArFunctor.getCPtr(f), f);
  }

  public SWIGTYPE_p_std__setT_int_t findPendingItems(String item) {
    return new SWIGTYPE_p_std__setT_int_t(AriaJavaJNI.ArSoundsQueue_findPendingItems(swigCPtr, this, item), true);
  }

  public void removePendingItems(String item, ArSoundsQueue.ItemType type) {
    AriaJavaJNI.ArSoundsQueue_removePendingItems__SWIG_0(swigCPtr, this, item, type.swigValue());
  }

  public void removePendingItems(String data) {
    AriaJavaJNI.ArSoundsQueue_removePendingItems__SWIG_1(swigCPtr, this, data);
  }

  public void removePendingItemsByPriority(int priority) {
    AriaJavaJNI.ArSoundsQueue_removePendingItemsByPriority(swigCPtr, this, priority);
  }

  public void removePendingItemsByPriorityWithType(int priority, ArSoundsQueue.ItemType type) {
    AriaJavaJNI.ArSoundsQueue_removePendingItemsByPriorityWithType(swigCPtr, this, priority, type.swigValue());
  }

  public void removePendingItemsByType(ArSoundsQueue.ItemType type) {
    AriaJavaJNI.ArSoundsQueue_removePendingItemsByType(swigCPtr, this, type.swigValue());
  }

  public void removeItems(int priority) {
    AriaJavaJNI.ArSoundsQueue_removeItems__SWIG_0(swigCPtr, this, priority);
  }

  public void removeItems(ArSoundsQueue.Item item) {
    AriaJavaJNI.ArSoundsQueue_removeItems__SWIG_1(swigCPtr, this, ArSoundsQueue.Item.getCPtr(item), item);
  }

  public String nextItemByType(ArSoundsQueue.ItemType type) {
    return AriaJavaJNI.ArSoundsQueue_nextItemByType(swigCPtr, this, type.swigValue());
  }

  public String nextItemByPriority(int priority) {
    return AriaJavaJNI.ArSoundsQueue_nextItemByPriority(swigCPtr, this, priority);
  }

  public String nextItemByTypeAndPriority(ArSoundsQueue.ItemType type, int priority) {
    return AriaJavaJNI.ArSoundsQueue_nextItemByTypeAndPriority(swigCPtr, this, type.swigValue(), priority);
  }

  public void setSpeakCallback(SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t cb) {
    AriaJavaJNI.ArSoundsQueue_setSpeakCallback(swigCPtr, this, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(cb));
  }

  public void setInterruptSpeechCallback(ArFunctor cb) {
    AriaJavaJNI.ArSoundsQueue_setInterruptSpeechCallback(swigCPtr, this, ArFunctor.getCPtr(cb), cb);
  }

  public void setPlayFileCallback(SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t cb) {
    AriaJavaJNI.ArSoundsQueue_setPlayFileCallback(swigCPtr, this, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(cb));
  }

  public void setPlayWavFileCallback(SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t cb) {
    AriaJavaJNI.ArSoundsQueue_setPlayWavFileCallback(swigCPtr, this, SWIGTYPE_p_ArRetFunctor2T_bool_char_const_p_char_const_p_t.getCPtr(cb));
  }

  public void setInterruptFileCallback(ArFunctor cb) {
    AriaJavaJNI.ArSoundsQueue_setInterruptFileCallback(swigCPtr, this, ArFunctor.getCPtr(cb), cb);
  }

  public void setInterruptWavFileCallback(ArFunctor cb) {
    AriaJavaJNI.ArSoundsQueue_setInterruptWavFileCallback(swigCPtr, this, ArFunctor.getCPtr(cb), cb);
  }

  public void speak(String str) {
    AriaJavaJNI.ArSoundsQueue_speak(swigCPtr, this, str);
  }

  public void play(String filename) {
    AriaJavaJNI.ArSoundsQueue_play(swigCPtr, this, filename);
  }

  public ArSoundsQueue.Item createDefaultSpeechItem(String speech) {
    return new ArSoundsQueue.Item(AriaJavaJNI.ArSoundsQueue_createDefaultSpeechItem__SWIG_0(swigCPtr, this, speech), true);
  }

  public ArSoundsQueue.Item createDefaultSpeechItem() {
    return new ArSoundsQueue.Item(AriaJavaJNI.ArSoundsQueue_createDefaultSpeechItem__SWIG_1(swigCPtr, this), true);
  }

  public ArSoundsQueue.Item createDefaultFileItem(String filename) {
    return new ArSoundsQueue.Item(AriaJavaJNI.ArSoundsQueue_createDefaultFileItem__SWIG_0(swigCPtr, this, filename), true);
  }

  public ArSoundsQueue.Item createDefaultFileItem() {
    return new ArSoundsQueue.Item(AriaJavaJNI.ArSoundsQueue_createDefaultFileItem__SWIG_1(swigCPtr, this), true);
  }

  public void setDefaultPlayConditionCB(ArRetFunctor_Bool f) {
    AriaJavaJNI.ArSoundsQueue_setDefaultPlayConditionCB(swigCPtr, this, ArRetFunctor_Bool.getCPtr(f), f);
  }

  public SWIGTYPE_p_void runThread(SWIGTYPE_p_void arg) {
    long cPtr = AriaJavaJNI.ArSoundsQueue_runThread(swigCPtr, this, SWIGTYPE_p_void.getCPtr(arg));
    return (cPtr == 0) ? null : new SWIGTYPE_p_void(cPtr, false);
  }

  public final static class ItemType {
    public final static ArSoundsQueue.ItemType SPEECH = new ArSoundsQueue.ItemType("SPEECH");
    public final static ArSoundsQueue.ItemType SOUND_FILE = new ArSoundsQueue.ItemType("SOUND_FILE");
    public final static ArSoundsQueue.ItemType SOUND_DATA = new ArSoundsQueue.ItemType("SOUND_DATA");
    public final static ArSoundsQueue.ItemType OTHER = new ArSoundsQueue.ItemType("OTHER");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static ItemType swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + ItemType.class + " with value " + swigValue);
    }

    private ItemType(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private ItemType(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private ItemType(String swigName, ItemType swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static ItemType[] swigValues = { SPEECH, SOUND_FILE, SOUND_DATA, OTHER };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
