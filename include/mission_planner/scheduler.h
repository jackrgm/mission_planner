#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "mission_planner/trail.h"
#include "mission_planner/node.h"

namespace mission_planner {
  // TODO add schedule method returning model of waypoints visited/unvisited
  class Scheduler
  {
  public:
    Scheduler();
    Scheduler(Trail t, Node &n);
    ~Scheduler();

    enum SchedulerState
    {
      ABORTED,
      BEGIN,
      FINISHED,
      GOALACHIEVED,
      GOALSENT,
      NOTSTARTED,
      PAUSED,
      STOPPING_ROBOT
    };

    void abort(Node &n);
    bool getDirection() const;
    int getLastVisited() const;
    bool getPaused() const;
    int getProgress();
    SchedulerState getState();
    int getTarget() const;
    double getTargetLat() const;
    double getTargetLon() const;
    double getTargetOrient() const;
    void pause(Node &n);
    int process(Node &n);
    void reverse(Node &n);

  private:
    //bool updateTarget(Node &n);
    bool metTarget(Node& n);
    void sendTargetGoal(Node &n) const;

    Trail trailcopy;
    bool paused = false;
    int progress = 0;
    bool reversed = false;
    SchedulerState state = NOTSTARTED;
    SchedulerState prev_state = NOTSTARTED;
    int target = -1;

	// TODO This should be configurable, maybe as a constructor param.
    double target_threshold_yaw = 0.6;
    double target_threshold_distance_metres = 3;
  };
}
#endif
