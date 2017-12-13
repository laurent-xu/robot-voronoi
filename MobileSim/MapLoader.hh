
/*  
    Copyright (C) 2005, ActivMedia Robotics LLC <http://www.activmedia.com>
    Copyright (C) 2006-2010 MobileRobots, Inc. <http://www.mobilerobots.com>
    Copyright (C) 2011-2015 Adept Technology

    This is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _EP_STAGE_MAP_LOADER_HH_
#define _EP_STAGE_MAP_LOADER_HH_

#include <set>
#include <string>
#include <vector>
#include "stage.h"
#include "ArFunctor.h"
#include "ariaUtil.h"
#include "ArGPSCoords.h"

class ArMap;
class ArMapObject;
class RobotInterface;

class MapLoadedInfo {
public:
  double min_x;
  double min_y;
  double max_x;
  double max_y;
  double home_x;
  double home_y;
  double home_th;
  bool have_home;
  std::string filename;
  int status; ///< 0=not reloaded, already loaded; 1=loaded; 2=error, file not found
  ArMap *map;
  MapLoadedInfo() : min_x(0), min_y(0), max_x(0), max_y(0), home_x(0), home_y(0), home_th(0), have_home(false), status(1), map(0)
  {}
};

typedef ArFunctor1<MapLoadedInfo>* MapLoadedCallback;
  ///< minx, miny, maxx, maxy, homex, homey, hometh

/** Load data from an ActivMedia/MobileRobots map file into
 * a Stage world, and keep track of models created. This class contains
 * the methods to do so, as well as keeps track of in-progress loading (loading
 * can be performed incrementally to avoid blocking the program for too long while doing so).
 * Generally this class is meant to be a singleton (MobileSim should just keep one instance to track loading).
 */
class MapLoader
{
private:
  stg_world_t *world;
  //StageInterface *interface;
  std::string mapfile;
  MapLoadedCallback callback;
  static const double ReflectorThickness; ///< meters
  std::set<stg_model_t*> mapModels; ///< all models created by loading maps
  ArMap *map; ///< points to an ArMap object while we are using it to load a map
  bool created_map; ///< we created map, and can delete it.
  bool loading; ///< true while we are in the process of loading a map, false when done.


  stg_polygon_t *polys;
  size_t npolys;
  size_t nlines;
  size_t nlines_loaded;
  std::vector<ArLineSegment>::const_iterator map_lines_iter;
  stg_point_t *points;
  size_t npoints;
  size_t npoints_loaded;
  std::vector<ArPose>::const_iterator map_points_iter;
  stg_model_t *new_map_model;

  ArLLACoords mapOriginLLA;

public:

  MapLoader(stg_world_t *_world = NULL) :
    world(_world), map(NULL), created_map(false), loading(false), haveMapOriginLLA(false)
  { }

  void setWorld(stg_world_t *_world) {
    cancelLoad();
    world = _world;
  }

  virtual ~MapLoader();
    
  std::string getMapName() { return mapfile; }

  bool haveMapOriginLLA;
  ArLLACoords getMapOriginLLA() { return mapOriginLLA; }

  /** Prepare to load a new map from the given file.
      Clears out any models created from a previous map from
      stage.  Call process() to make progress on actually loading the data and creating models in the stage world.
      If called while still loading another map, that map is closed.  If the given map file is already loaded (i.e. is not
      a new name, or file has not changed since last load), then loading is considered finished (and @a callback is invoked to
      indicate this).
      @param callback If not NULL, call this callback when map is done loading.
      @param errormsg if not NULL, place any error messages in this string
      @return true if ready to load, false on error (error opening file or preparing to read)
    */
  bool newMap(const std::string& newmapfile, RobotInterface *requestor, MapLoadedCallback callback = NULL, std::string *errormsg = NULL);

  bool newMap(ArMap *newmap, MapLoadedCallback callback = NULL);

  ArMap *getMap() { return map; }
private:
  bool newMap(MapLoadedCallback cb);

public:
  /// Proceed to load data from an open map file, or do nothing if map loading is not in progress.
  /// @param maxTime if not 0, stop loading when this amount of time has passed. Loading will resume on next call to process(). If 0, no time limit.
  bool process(unsigned int maxTime = 0) { /* XXX TODO */ maxTime = maxTime;  return true; };

  virtual void cancelLoad();

private:
  /// Check whether the given map file name is new or the map file has changed since the last time it was loaded.
  /// @return true if new map file or file changed, otherwise return false.
  bool shouldReloadMap(const std::string& newmapfile);

  struct ObjectClass {
    std::string name;
    bool obstacle;
    stg_laser_return_t laser_return;
    bool sonar_return;
    ObjectClass() : obstacle(true), laser_return(1), sonar_return(1) {}
    ObjectClass(std::string _name) : name(_name), obstacle(true), laser_return(1), sonar_return(1) {}
  };
  stg_model_t* loadReflector(ArMapObject* cairnObj, stg_model_t* map_model, stg_laser_return_t laser_return);
  stg_model_t* loadBoxObstacle(ArMapObject* cairnObj, stg_model_t* map_model, ObjectClass objclass);

  void invokeMapLoadedCallback(MapLoadedCallback cb, int status, std::string filename, ArMap* map) const;
  void reset();
};

#endif
