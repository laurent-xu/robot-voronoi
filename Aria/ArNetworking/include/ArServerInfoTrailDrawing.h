#ifndef ARSERVERINFOTRAILDRAWING
#define ARSERVERINFOTRAILDRAWING

#include "Aria.h"
#include "ArNetworking.h"
#include <deque>

/** Stores a history of ArPose objects, and provides them as drawings to a
 * client (such as MobileEyes) via ArServerInfoDrawings.  Positions are obtained
 * via the supplied functor, so any object providing positions as ArPose objects
 * may be used as the source of the trail.  This functor is called when
 * triggered by calling pull(). 
 * 
 * @todo A way to automatically lock a mutex on the target object when triggered.
 * @todo Don't add new points that are the same or very close to existing points in buffer.
 */
class ArServerInfoTrailDrawing 
{
public:

  ArServerInfoTrailDrawing(const char *name, ArRetFunctor<ArPose> *func, size_t max, const ArDrawingData &dd, ArServerInfoDrawings &drawingServer, unsigned int interval = 1, float minDist = 0) : 
      myFunc(func), 
      myNumPoints(max), 
      myDrawingData(dd), 
      myDrawFunc(this, &ArServerInfoTrailDrawing::draw),
      myPullFunc(this, &ArServerInfoTrailDrawing::pull), 
      myName(name), 
      myInterval(interval), 
      myCounter(0),
      myMinDist(minDist)
  {
    drawingServer.addDrawing(&myDrawingData, name, &myDrawFunc);
  }

  /** Add this method as a callback to an object (e.g. ArRobot task), or call it
   * when you want a new position added to the trail. You can get a functor
   * to use as a callback with getPullFunc().  Example:
   * @code
   *   robot.addSensorInterpTask("Trail", 20, trailDrawings.getPullFunc());
   * @endcode
   * 
   * In Python, you can reference the function directly:
   * @code
   *   robot.addSensorInterpTask('Trail', 20, trailDrawings.pull)
   * @endcode
   */
  void pull() 
  {
    if(++myCounter < myInterval) 
      return;
    myCounter = 0;
    ArPose p = myFunc->invokeR();
    if(!myPoints.empty() && p.findDistanceTo(myPoints.back()) < myMinDist)
      return;
    while(myPoints.size() >= myNumPoints)
      myPoints.pop_front();
    myPoints.push_back(p);
  } 
  
  ArFunctor *getPullFunc() { return &myPullFunc; }

  void setPullInterval(unsigned int i) { myInterval = i; }
  void setMinDist(float d) { myMinDist = d; }
  void setMaxNumPoints(unsigned int n) { myNumPoints = n; }

protected:

  void draw(ArServerClient *client, ArNetPacket *pkt)
  {
    ArNetPacket reply;
    reply.byte4ToBuf(myPoints.size());
    for(std::deque<ArPose>::const_iterator i = myPoints.begin(); i != myPoints.end(); ++i)
    {
      reply.byte4ToBuf((int)(i->getX()));
      reply.byte4ToBuf((int)(i->getY()));
    }
    client->sendPacketUdp(&reply);
  }

  ArRetFunctor<ArPose> *myFunc;
  size_t myNumPoints;
  ArDrawingData myDrawingData;
  std::deque<ArPose> myPoints;
  ArFunctor2C<ArServerInfoTrailDrawing, ArServerClient*, ArNetPacket*> myDrawFunc;
  ArFunctorC<ArServerInfoTrailDrawing> myPullFunc;
  std::string myName;
  unsigned int myInterval;
  unsigned int myCounter;
  float myMinDist;
};

#endif
