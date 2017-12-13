/*

  Copyright (C) 2005, ActivMedia Robotics, LLC
  Copyright (C) 2006-2010 MobileRobots, Inc.
  Copyright (C) 2011-2015 Adept Technology Inc.

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

*/

/* TODO separate out map loading into a new "World" interface. */

// to maybe get extensions like sincos():
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif
#ifndef __USE_GNU
#define __USE_GNU 1
#endif

#include <math.h>
#include <assert.h>
#include "stage.h"
#include "MapLoader.hh"
#include "RobotInterface.hh"
#include "ArMap.h"
#include "ariaUtil.h"


const double MapLoader::ReflectorThickness = 0.0200; // meters (=2cm).



#define PRINT_NUM_POINTS_CREATED 100000
#define PRINT_NUM_LINES_CREATED  10000

MapLoader::~MapLoader()
{
  if(map && created_map)
    delete map;
}


void MapLoader::cancelLoad() {
  reset();
}




bool MapLoader::newMap(const std::string& newmapfile, RobotInterface *requestor, MapLoadedCallback cb, std::string *errorMsg)
{
  if(!shouldReloadMap(newmapfile))
  {
    if(requestor) requestor->warn("Not reloading map file \"%s\', it has not changed since last load.", newmapfile.c_str());
    else print_warning("Not reloading map file \"%s\', it has not changed since last load.", newmapfile.c_str());
    invokeMapLoadedCallback(cb, false, newmapfile, NULL);
    return true;
  }

  /// ??? @todo ? Get points and lines change timestamps from ArMap and check against saved times.
  ///  (Though note that this will create a bug, that modifying reflectors or ?
  //obstacle cairns will be ignored)

  reset();

  ArTime timer;

  if(!map) {
    map = new ArMap();
    created_map = true;
  }

  char errbuf[128];
  if(!map->readFile(newmapfile.c_str(), errbuf, 127))
  {
    if(errorMsg) *errorMsg = errbuf;
    return false;
  }
  //print_debug("Took %d msec to read map file", timer.mSecSince());
  return newMap(cb);
}

bool MapLoader::newMap(ArMap *newmap, MapLoadedCallback cb)
{
  reset();
  map = newmap;
  return newMap(cb);
}

bool MapLoader::newMap(MapLoadedCallback cb)
{
  loading = true;
  std::string newmapname = map->getFileName();
  callback = cb;
  mapfile = newmapname;


    // TODO move all that follows into process()

    // TODO build a new matrix at the right resolution and size and use it to
    // filter out redundant points for that resolution (avoid too much data in
    // stg_point_t array for instance) as we get
    // data from ArMap, then add robots from old matrix, then swap in new
    // matrix. This should be doable over several process() calls, and avoid
    // having to resize the world.

  // Load map data (this is slightly time consuming, so don't lock world yet)
  stg_polygon_t *polys = NULL;
  size_t npolys = 0;
  size_t nlines = 0;
  bool loadedData = false;

  ArTime timer;


  // load lines from map

  nlines = map->getLines()->size();
  if(nlines > 0)
  {
    polys = stg_polygons_create(nlines);
    for(std::vector<ArLineSegment>::const_iterator i = map->getLines()->begin();
    i != map->getLines()->end(); ++i)
    {
      // make an infintessimally thin rectangle for this line. convert mm to m
      stg_point_t v1 = {(*i).getX1() / 1000.0, (*i).getY1() / 1000.0};
      stg_point_t v2 = {(*i).getX2() / 1000.0, (*i).getY2() / 1000.0};
      stg_polygon_append_points(&polys[npolys], &v1, 1);
      stg_polygon_append_points(&polys[npolys], &v1, 1);
      stg_polygon_append_points(&polys[npolys], &v2, 1);
      stg_polygon_append_points(&polys[npolys], &v2, 1);
      ++npolys;
#ifdef PRINT_NUM_LINES_CREATED
      if(npolys > 0 && npolys % PRINT_NUM_LINES_CREATED == 0) printf("MobileSim map loader: At %lu lines...\n", npolys);
#endif
    }
    loadedData = true;
  }


  //print_debug("Took %d msec to convert line data from ArMap to stage objects.", timer.mSecSince());

  stg_point_t *points = NULL;
  size_t npoints = 0;

  // load points from map

  timer.setToNow();
  npoints = map->getPoints()->size();
  if(npoints > 0)
  {
    points = stg_points_create(npoints);
    stg_point_t* p = points;
    size_t count = 0;
    for(std::vector<ArPose>::const_iterator i = map->getPoints()->begin();
    i != map->getPoints()->end(); ++i)
    {
      p->x = (double) (*i).getX() / 1000.0;     // convert mm to m
      p->y = (double) (*i).getY() / 1000.0;
      ++p;
#ifdef PRINT_NUM_POINTS_CREATED
      if(count > 0 && count % PRINT_NUM_POINTS_CREATED == 0) printf("MobileSim map loader: At %lu points...\n", count);
      ++count;
#endif
    }

    loadedData = true;
  }


  //print_debug("Took %d msec to convert point data from ArMap to stage objects.", timer.mSecSince());

  if(!loadedData)
  {
    stg_print_warning("MobileSim: No obstacle data loaded from map file \"%s\"!", mapfile.c_str());
    //if(!loadPoints) stg_print_warning("MobileSim: Requested not to load point data, try enabling.");
    //if(!loadLines) stg_print_warning("MobileSim: Requested not to load line data, try enabling.");
  }


  // Load origin georeference
  haveMapOriginLLA = map->hasOriginLatLongAlt();
  if(haveMapOriginLLA)
  {
    mapOriginLLA.setX(map->getOriginLatLong().getX());
    mapOriginLLA.setY(map->getOriginLatLong().getY());
    mapOriginLLA.setZ(map->getOriginAltitude());
    stg_print_msg("MobileSim: Map has OriginLatLon point, will be able to send simulated GPS coordinates if requested.");
  }

  // Clear any existing map models
  for(std::set<stg_model_t*>::const_iterator i = mapModels.begin(); i != mapModels.end(); i++)
  {
    assert(*i);
    stg_world_remove_model(world, *i);
    //printf("clearMap: destroying model \"%s\"...\n", stg_model_get_token(*i));
    stg_model_destroy(*i);
  }
  mapModels.clear();

  // find a unique id number
  stg_id_t id = 0;
  for(id = 0; stg_world_get_model(world, id) != NULL && id <= STG_ID_T_MAX; ++id)
    ;
  if(id == STG_ID_T_MAX)
  {
    stg_print_error("MobileSim: !!! too many models in the world, can't create a new one.");
    return false;
  }

    // create model TODO use stg_world_new_model instead!
  // set as a background figure, so it isn't redrawn every time the robot moves
  // or whatever.
  stg_model_t* map_model = stg_model_create(world, NULL, id, newmapname.c_str(), "model", "model", 0, NULL, TRUE);
  //print_debug("MapLoader: new model for map is %p", map_model);

  // store the map file name
  stg_model_set_property(map_model, "source", (void*)mapfile.c_str(), mapfile.size()+1);

  // set color
  stg_color_t mapcolor = stg_lookup_color("dark gray");
  if(mapcolor == 0xFF0000) mapcolor = 0; // black if not found
  stg_model_set_property(map_model, "color", &mapcolor, sizeof(mapcolor));

  // set size
  double maxX_mm = 0;
  double minX_mm = 0;
  double maxY_mm = 0;
  double minY_mm = 0;
  if(npoints > 0 && nlines > 0)
  {
    maxX_mm = fmax(map->getLineMaxPose().getX(), map->getMaxPose().getX());
    minX_mm = fmin(map->getLineMinPose().getX(), map->getMinPose().getX());
    maxY_mm = fmax(map->getLineMaxPose().getY(), map->getMaxPose().getY());
    minY_mm = fmin(map->getLineMinPose().getY(), map->getMinPose().getY());
  }
  else if(npoints > 0)
  {
    maxX_mm = map->getMaxPose().getX();
    minX_mm = map->getMinPose().getX();
    maxY_mm = map->getMaxPose().getY();
    minY_mm = map->getMinPose().getY();
  }
  else if(nlines > 0)
  {
    maxX_mm = map->getLineMaxPose().getX();
    minX_mm = map->getLineMinPose().getX();
    maxY_mm = map->getLineMaxPose().getY();
    minY_mm = map->getLineMinPose().getY();
  }
  stg_size_t size;
  size.x = maxX_mm/1000.0 - minX_mm/1000.0; //mm to m
  size.y = maxY_mm/1000.0 - minY_mm/1000.0; //mm to m
  stg_print_msg("New world from loading map \"%s\" will be %f x %f meters in size.", mapfile.c_str(), size.x, size.y);
  stg_model_set_size(map_model, size);

  // but don't scale it to that size, lines and points are already at the right
  // places for correct scale
  stg_model_set_scaling(map_model, FALSE);

  // set origin offset
/*
  stg_pose_t offset;
  offset.x = (size.x / 2.0) + (minX_mm / 1000.0);
  offset.y = (size.y / 2.0) + (minY_mm / 1000.0);
  offset.a = 0;
  stg_model_set_origin(map_model, offset);
*/

  // Turn off grid
  int grid = 0;
  stg_model_set_property(map_model, "grid", &grid, sizeof(int));

  // Make it unmovable
  int movemask = 0; 
  stg_model_set_property(map_model, "mask", &movemask, sizeof(int));

  // store creation time and file source
  stg_model_set_property(map_model, "source", (void*)newmapname.c_str(), newmapname.size()+1);

  time_t tm = time(NULL);
  stg_model_set_property(map_model, "creation_time", &tm, sizeof(tm));


  // store map data. 
  timer.setToNow();
  if(polys)
  {
    //puts("XXX Storing polygons...");
    stg_model_init_polygons(map_model, polys, npolys);
    free(polys);  // it was copied by stg_model_init_polygons
  }
  //print_debug("Took %d msec to store model polygons in world.", timer.mSecSince());

  timer.setToNow();
  if(points)
  {
    //puts("XXX Storing points...");
    stg_model_init_points(map_model, points, npoints);
    stg_points_destroy(points); // it was copied by stg_model_init_points
  }

  //print_debug("Took %d msec to store model points in world.", timer.mSecSince());


  // Remember this model
  mapModels.insert(map_model);

  // Add model to world. Will be mapped in.
  //timer.setToNow();
  stg_world_add_model(world, map_model);
  //printf("XXX Took %d msec to add model to world.\n", timer.mSecSince());
  //printf("loadMap: added map model \"%s\" to world.\n", stg_model_get_token(map_model));


  // Check special simulator attributes of custom map object type definitions,
  // and create models for objects as neccesary.
  // TODO check Color0 and Color1, SonarReflect.
  std::map<std::string, ObjectClass> object_classes;

  // Built in reflector type always has a high laser retun value by default
  ObjectClass reflector_class("Reflector");
  reflector_class.laser_return = 2;
  object_classes["Reflector"] = reflector_class;

  for(std::list<ArArgumentBuilder*>::const_iterator i = map->getMapInfo()->begin(); i != map->getMapInfo()->end(); ++i)
  {
    const char *type_name = (*i)->getArg(1);
    if( strncmp(type_name, "Name=", 5) != 0 )
    {
      //stg_print_warning("MobileSim: First MapInfo attribute is not \"Name\", skipping.");
      continue;
    }
    type_name += strlen("Name="); // skip past the "Name=" prefix

    ObjectClass new_class(type_name);

    const char *shape = (*i)->getArg(0);
    // XXX BoundaryType not implemented yet
    if(strcmp(shape, "SectorType") != 0)
      continue;

    // Reflectors can be built-in reflectors, or have a name that
    // previous versions of MobileSim interpreted as automatically being
    // reflectors.
    int val = 1;
    const char* endTag = strrchr(type_name, '.');
    if(   strcmp( type_name, "Sim.Reflector") == 0 
      || strcmp(  type_name, "Reflector") == 0
      || (endTag && strcmp(endTag, ".Reflect") == 0)
    )
    {
      new_class.laser_return = 2;
    }

    // Check remaining attributes for special simulator things
    for(size_t a = 2; a < (*i)->getArgc(); ++a)
    {
      char buf[256];
      ArUtil::stripQuotes(buf, (*i)->getArg(a), 256);

      // Reflective to laser?
      new_class.laser_return = 0;
      if(strncmp(buf, "Sim.LaserReflect=", strlen("Sim.LaserReflect=")) == 0)
      {
        if(strncmp(buf, "Sim.LaserReflect=no", strlen("Sim.LaserReflect=no")) == 0 || strncmp(buf, "Sim.LaserReflect=false", strlen("Sim.LaserReflect=false")) == 0)
          new_class.laser_return = 0;
        else
          new_class.laser_return = atoi( buf + strlen("Sim.LaserReflect=") ) + 1;   // Need to add one because Stage starts highly reflective objects at 2, but SICK/Aria at 1
        stg_print_msg("MobileSim: Will use reflection value %d for objects with type %s (from Sim.LaserReflect attribute in MapInfo)", new_class.laser_return, type_name);
      }

      // To sonar?
      if(strncmp(buf, "Sim.SonarReflect=no", strlen("Sim.SonarReflect=no")) == 0 || strncmp(buf, "Sim.SonarReflect=false", strlen("Sim.SonarReflect=false")) == 0)
      {
        new_class.sonar_return = false;
        stg_print_msg("MobileSim: Objects of type %s %s be visible to sonar (from Sim.SonarReflect attribute in MapInfo)", type_name, new_class.sonar_return?"will":"will not");
      }


      new_class.obstacle = false;

      // Obstacle to robot?
      if(strcasecmp(buf, "Sim.Obstacle=yes") == 0 || strcasecmp(buf, "Sim.Obstacle=true") == 0)
      {
        stg_print_msg("MobileSim: Objects of type %s will be represented as obstacles (from Sim.Obstacle attribute in MapInfo for %s)", type_name, type_name);
        new_class.obstacle = true; 
      }

      // Non-obstacle to robot?
      if(strcasecmp(buf, "Sim.Obstacle=no") == 0 || strcasecmp(buf, "Sim.Obstacle=false") == 0)
      {
        stg_print_msg("MobileSim: Objects of type %s will be represented as non-obstacle objects (from Sim.Obstacle attribute in MapInfo for %s)", type_name, type_name);
        new_class.obstacle = false;
      }
    }

    object_classes[type_name] = new_class;
  }

  // Create special objects for certain Cairn objects.
  // Reflector: make a line in the map with bright reflectance. 
  // Sim.BoxObstacle: Make a box that the user can move. 
  for(std::list<ArMapObject*>::const_iterator i = map->getMapObjects()->begin(); i != map->getMapObjects()->end(); ++i)
  {
    ArMapObject* obj = (*i);
    if(obj == NULL) continue;

    stg_model_t* model = NULL;

    bool builtinReflector = false;

    // XXX this needs to be refactored a bit, probably eliminate the seperate
    // loadReflector and LoadBoxObstacle functions, just set unique properties
    // seperately.

    std::map<std::string, ObjectClass>::iterator c = object_classes.find(obj->getType());
    if(c != object_classes.end())
    {
      // Built-in Reflector objects have special fixed properties (color, shape, etc.) 
      if(strcmp(obj->getType(), "Reflector") == 0 || c->second.laser_return > 1) 
      {
        builtinReflector = true;
        model = loadReflector(obj, map_model, c->second.laser_return);
      }
      // Is the object of a type (class) that should be an obstacle?
      else if (c->second.obstacle)
      {
        model = loadBoxObstacle(obj, map_model, c->second);
      }
      // Otherwise, ignore it.
    }

    // TODO support line-shaped obstacles, and reflective thngs that aren't also
    // obstacles (they're phantom reflectors)

    if(model == NULL) continue;  // no simulator obstacle was created for this map object

    // store file it was loaded from and current time
    stg_model_set_property(model, "source", (void*)newmapname.c_str(), newmapname.size()+1);
    time_t t = time(NULL);
    stg_model_set_property(model, "creation_time", &t, sizeof(t));

    mapModels.insert(model);
  }


  // resize world 
  /// @todo Only resize if it got bigger. Also should optimize this, it takes forever.
  timer.setToNow();
  stg_world_resize_to_contents(world, 10);
  //print_debug("Took %d msec to resize world to new contents.", timer.mSecSince());

  //stg_world_unlock(world);


  if(callback)
    invokeMapLoadedCallback(callback, 1, mapfile, map);

  reset();

  return true;
}




/** Does not lock world */
stg_model_t* MapLoader::loadReflector(ArMapObject* obj, stg_model_t* /*map_model*/, stg_laser_return_t laser_return)
{
  if(!obj->hasFromTo()) 
  {
    stg_print_warning("MobileSim: Found a Reflector Cairn in the map, but the line has no 'from' and 'to' position; skipping.");
    return NULL;
  }


  /*
  stg_print_msg("Found a Reflector in the map file at pose (%fmm,%fmm); a line from (%fmm,%fmm) to (%fmm,%fmm)",
  obj->getPose().getX(), obj->getPose().getY(), 
  obj->getFromPose().getX(), obj->getFromPose().getY(), 
  obj->getToPose().getX(), obj->getToPose().getY());
  */

  // Make a very thin magenta box aligned with the reflector line

  stg_pose_t reflector_pose;




  double line_x1 = (obj->getFromPose().getX() / 1000.0);
  double line_x2 = (obj->getToPose().getX() / 1000.0);
  double line_y1 = (obj->getFromPose().getY() / 1000.0);
  double line_y2 = (obj->getToPose().getY() / 1000.0);
  double line_dx = line_x2 - line_x1;
  double line_dy = line_y2 - line_y1;


  // Turn it into a box with some thickness
  double line_angle = atan( fabs(line_dy) / fabs(line_dx) );
  double reflector_theta = fabs(line_angle + reflector_pose.a);
  double disp_x = fabs(ReflectorThickness * sin(reflector_theta));
  double disp_y = fabs(ReflectorThickness * cos(reflector_theta));

  // If an object in stage has size (x or y) == 0, then it become
  // infinitely large!
  if(disp_x <= 0.000001) disp_x = 0.000001;
  if(disp_y <= 0.000001) disp_y = 0.000001;

  // Use cairn pose:
  //reflector_pose.x = obj->getPose().getX() / 1000.0;
  //reflector_pose.y = obj->getPose().getY() / 1000.0;
  //reflector_pose.a = obj->getPose().getTh(); // Specifically *don't* use rotation, ARIA doesn't.
  //reflector_pose.a = 0;

  // Alternatively, figure pose from line endpoints:
  reflector_pose.x = line_x1 + (line_dx / 2.0);
  reflector_pose.y = line_y1 + (line_dy / 2.0);
  reflector_pose.a = 0;

  // Find a unique name
  char token[32];
  memset(token, 0, 32);
  strcat(token, "Reflector:0");
  int i = 0;
  for(; stg_world_model_name_lookup(world, token) != NULL; i++)
  {
    snprintf(token, 32, "Reflector:%i", i);
  }

  // find a unique id number
  stg_id_t id = 0;
  for(id = 0; stg_world_get_model(world, id) != NULL; id++)
    ;

  // create reflector model.  Note, not making map a parent, this causes
  // problems currently. TODO use stg_world_new_model instead.
  stg_model_t* reflector_model = stg_model_create(world, NULL, id, token, "model", "model", i, NULL, FALSE);
  assert(reflector_model);


  // set size and position
  stg_size_t reflector_size = { fabs(line_dx) + disp_x, fabs(line_dy) + disp_y };
  stg_model_set_size(reflector_model, reflector_size);
  stg_model_set_pose(reflector_model, reflector_pose);

  // bright laser return
  stg_model_set_property(reflector_model, "laser_return", &laser_return, sizeof(laser_return));

  // color
  stg_color_t color = stg_lookup_color("magenta");
  stg_model_set_property(reflector_model, "color", &color, sizeof(color));

  // shape
  stg_polygon_t* reflector_polys = stg_polygons_create(1);
  stg_point_t v1 = { line_x1, line_y1 };
  stg_point_t v2 = { line_x1 + disp_x, line_y1 - disp_y };
  stg_point_t v3 = { line_x2 + disp_x, line_y2 - disp_y };
  stg_point_t v4 = { line_x2, line_y2 };
  stg_polygon_append_points(&reflector_polys[0], &v1, 1);
  stg_polygon_append_points(&reflector_polys[0], &v2, 1);
  stg_polygon_append_points(&reflector_polys[0], &v3, 1);
  stg_polygon_append_points(&reflector_polys[0], &v4, 1);
  stg_model_init_polygons(reflector_model, reflector_polys, 1);

  int reflector_movemask = 1|2; // movable|rotatable
  int reflector_outline = 0;    // no outline
  stg_model_set_property(reflector_model, "mask", &reflector_movemask, sizeof(int));
  stg_model_set_property(reflector_model, "outline", &reflector_outline, sizeof(int));

  stg_world_add_model(world, reflector_model);
  //printf("loadMap: added reflector model \"%s\" to world.\n", stg_model_get_token(reflector_model));
  return reflector_model;
}

/** Does not lock world */
stg_model_t* MapLoader::loadBoxObstacle(ArMapObject* obj, stg_model_t* /*map_model*/, ObjectClass object_class)
{
  // Todo, look for a MapInfo declaration that defines properties like color for this
  // type of obstacle. (e.g. Sim.SonarOnly, Sim.Invisible, etc.)
  if(!obj->hasFromTo())
  {
    stg_print_warning("MobileSim: Found a Sim.BoxObstacle Cairn in the map, but the line has no 'from' and 'to' position. Skipping.");
    return NULL;
  }

  stg_print_msg("MobileSim: Found a Box Obstacle in the map file at pose (%.0fmm,%.0fmm,%.0fdeg); a box from (%.0fmm,%.0fmm) to (%.0fmm,%.0fmm)",
  obj->getPose().getX(), obj->getPose().getY(), obj->getPose().getTh(), 
  obj->getFromPose().getX(), obj->getFromPose().getY(), 
  obj->getToPose().getX(), obj->getToPose().getY());

  // Find a unique name and id
  char token[32];
  memset(token, 0, 32);
  strcat(token, "Box:0");
  int i = 0;
  for(; stg_world_model_name_lookup(world, token) != NULL; i++)
  {
    snprintf(token, 32, "Box:%i", i);
  }

  // find a unique name
  stg_id_t id = 0;
  for(id = 0; stg_world_get_model(world, id) != NULL; id++)
    ;

  // create model. note, using null parent. making the map a parent causes
  // problems right now. TODO use stg_world_new_model instead!
  stg_model_t* box_model = stg_model_create(world, NULL, id, token, "model", "model", i, NULL, FALSE);


  // size
  stg_size_t box_size;
  box_size.x = (obj->getToPose().getX() - obj->getFromPose().getX()) / 1000.0;
  box_size.y = (obj->getToPose().getY() - obj->getFromPose().getY()) / 1000.0;
  stg_model_set_size(box_model, box_size);

  // pose. Map object pose is not used for some reason, it needs to be
  // positioned according to the "from" and "to" poses, in this slightly
  // complex way.
  stg_pose_t box_pose;
  double sinObjTh, cosObjTh;
  STG_SINCOS(DTOR(obj->getPose().getTh()), sinObjTh, cosObjTh);
  double x = (obj->getFromPose().getX() / 1000.0) + (box_size.x / 2.0);
  double y = (obj->getFromPose().getY() / 1000.0) + (box_size.y / 2.0);
  box_pose.x = (x * cosObjTh) - (y * sinObjTh);
  box_pose.y = (x * sinObjTh) + (y * cosObjTh);
  box_pose.a = DTOR(obj->getPose().getTh());
  stg_model_set_pose(box_model, box_pose);

  // color
  if(object_class.laser_return > 1)
  {
      stg_color_t color = stg_lookup_color("magenta");
      stg_model_set_property(box_model, "color", &color, sizeof(color));
  }
  else
  {
    stg_color_t color = stg_lookup_color("light green");
    stg_model_set_property(box_model, "color", &color, sizeof(color));
  }

  // shape
  stg_polygon_t* box_polys = stg_polygons_create(1);
  //stg_point_t v1 = { obj->getFromPose().getX()/1000.0, obj->getFromPose().getY()/1000.0 };
  //stg_point_t v2 = { obj->getToPose().getX()/1000.0, obj->getFromPose().getY()/1000.0 };
  //stg_point_t v3 = { obj->getToPose().getX()/1000.0, obj->getToPose().getY() / 1000.0 };
  //stg_point_t v4 = { obj->getFromPose().getX()/1000.0, obj->getToPose().getY()/1000.0 };
  stg_point_t v1 = { -box_size.x/2.0, -box_size.y/2.0 };   // actually equivalent... ?
  stg_point_t v2 = { box_size.x/2.0, -box_size.y/2.0 };
  stg_point_t v3 = { box_size.x/2.0, box_size.y/2.0 };
  stg_point_t v4 = { -box_size.x/2.0, box_size.y/2.0 };
  stg_polygon_append_points(&box_polys[0], &v1, 1);
  stg_polygon_append_points(&box_polys[0], &v2, 1);
  stg_polygon_append_points(&box_polys[0], &v3, 1);
  stg_polygon_append_points(&box_polys[0], &v4, 1);
  stg_model_init_polygons(box_model, box_polys, 1);

  int box_movemask = 1|2; // movable and rotatable
  int box_outline = 1;  // has a black outline
  stg_model_set_property(box_model, "mask", &box_movemask, sizeof(int));
  stg_model_set_property(box_model, "outline", &box_outline, sizeof(int));

  int box_obstacle_return = object_class.obstacle?1:0;
  stg_model_set_property(box_model, "obstacle_return", &box_obstacle_return, sizeof(box_obstacle_return));
  stg_model_set_property(box_model, "laser_return", &object_class.laser_return, sizeof(object_class.laser_return));
  int ranger_return = object_class.sonar_return?1:0;
  stg_model_set_property(box_model, "ranger_return", &ranger_return, sizeof(ranger_return));

  stg_world_add_model(world, box_model);
  //printf("loadMap: added box model \"%s\" to world.\n", stg_model_get_token(box_model));
  return box_model;
}

bool MapLoader::shouldReloadMap(const std::string& mapfile)
{
  struct stat filestat;
  int s = stat(mapfile.c_str(), &filestat);
  if(s != 0)
  return true;  // no way to know modification time, force reload
  bool hadModelsFromThatMap = false;
  for(std::set<stg_model_t*>::const_iterator i = mapModels.begin(); i != mapModels.end(); i++)
  {
    size_t len;
    char *source = (char*)stg_model_get_property(*i, "source", &len);
    if(source && len > 0 && mapfile == source)
    {
      // model came from this map file
      hadModelsFromThatMap = true;
      time_t *modelTime = (time_t*)stg_model_get_property_fixed(*i, "creation_time", sizeof(time_t));
      if(!modelTime) {
        stg_world_unlock(world);
        return true;  // no creation_time, no way to know if it's newer, always reload map file
      }
      if( difftime( filestat.st_mtime, *modelTime ) > 0.0) {
        //printf("map file has modification time %d (%s), newer than model %s's creation time %d (%s). must reload.\n", filestat.st_mtime, ctime(&filestat.st_mtime), stg_model_get_token(*i), *modelTime, ctime(modelTime));
        stg_world_unlock(world);
        return true; 
      }
    }
  }
  if(hadModelsFromThatMap) {
    return false;
  } else {
    return true;   // force reload if no models came from that map (it's a completely new map)
  }
}


void MapLoader::invokeMapLoadedCallback(MapLoadedCallback cb, int status, std::string filename, ArMap* map) const
{
  if(!cb) return;
  MapLoadedInfo info;
  if(map)
  {
    info.map = map;
    ArMapObject *home = map->findFirstMapObject(NULL, "RobotHome");
    if(!home)
      home = map->findFirstMapObject(NULL, "Dock");
    if(home)
    {
      info.have_home = true;
      info.home_x = home->getPose().getX();
      info.home_y = home->getPose().getY();
      info.home_th = home->getPose().getTh();
    }
    info.min_x = map->getLineMinPose().getX();
    info.min_y = map->getLineMinPose().getY();
    info.max_x = map->getLineMaxPose().getX();
    info.max_y = map->getLineMaxPose().getY();
  }
  info.filename = filename;
  info.status = status;
  cb->invoke(info);
}

void MapLoader::reset()
{
  loading = false;
  callback = NULL;
  if(map && created_map)
  {
    delete map;
  }
  map = NULL;
  polys = NULL;
  points = NULL;
  npolys = nlines = nlines_loaded = npoints = npoints_loaded = 0;
  new_map_model = NULL;
}
