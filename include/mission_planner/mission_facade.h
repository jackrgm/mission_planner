#ifndef MISSION_FACADE_H
#define MISSION_FACADE_H

#include "mission_planner/node.h"
#include "mission_planner/recorder.h"
#include "mission_planner/scheduler.h"
#include "mission_planner/trail.h"

// provides a simple api to the 'back end' for easy 'front end' implementation
class MissionFacade
{
  public:
  //MissionFacade(int argc, char** argv);
  MissionFacade();
  ~MissionFacade();
	
  // TODO check: probs need to add orientation too
  // may be worth looking up gps_goal + sending normal 2D nav goals for learning
  // also may be worth setting up sat_goal first as a learning mechanism
  void goalSend(double lat, double lon);
  void goalCancel();

  void trailAdd();
  void trailInsert();
  void trailRemove();
  void trailClear();

  void missionStart();
  void missionAbort();
  void missionReverse();
  void missionPause();

  void getMission();
  void getProgress();
  void getTarget();
  void getLastVisited();

  // Waypoint(node.getPos)
  // TrailAdd(Waypoint)
  //recorder.captureWaypoint maybe? + recorder.auto() for autoadd
  void capture();

  private:
  //Scheduler scheduler;
  //Node node;
  //Recorder recorder;
  //Trail trail;
};

#endif
