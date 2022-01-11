#include <iostream>
#include "mission_planner/scheduler.h"
#include "sat_goal/utility.h"
#include <cmath>
#include <math.h>

namespace mission_planner {
  Scheduler::Scheduler()
  {

  }

  // start a navigation mission
  Scheduler::Scheduler(Trail t, Node &n)
  {
	// Taking a shallow copy ensures trail modifications don't affect missions.
    trailcopy = t;
    target = 0;

    // Ensure the robot is not in the middle of something.
    prev_state = BEGIN;
    state = STOPPING_ROBOT;
  }

  Scheduler::~Scheduler()
  {

  }

  void Scheduler::reverse(Node &n)
  {
    prev_state = GOALACHIEVED;
    state = STOPPING_ROBOT;

    if (reversed)
    {
      reversed = false;
      target++;
    }
    else
    {
      reversed = true;
      target--;

      if(target < 0)
      {
        state = BEGIN;
      }
    }
  }

  void Scheduler::pause(Node &n)
  {
    if (state == NOTSTARTED || state == FINISHED || state == ABORTED) {
      return;
    }

    if (state == PAUSED)
    {
      if (target < 0)
      {
        state = BEGIN;
      }
      else
      {
        sendTargetGoal(n);
        state = GOALSENT;
      }
    }
    else
    {
      prev_state = PAUSED;
      state = STOPPING_ROBOT;
    }
  }

  void Scheduler::abort(Node &n)
  {
    if(state = FINISHED)
    {
      prev_state = NOTSTARTED;
    }
    else
    {
      prev_state = ABORTED;
    }

    target = -1;
    state = STOPPING_ROBOT;
  }

  int Scheduler::getProgress()
  {
    double total_visited = 0.0;
    double trailsize = trailcopy.getSize();

    for(int i = 0; i < trailcopy.getSize(); i++)
    {
      if(trailcopy.hasVisited(i))
      {
        total_visited++;
      }
    }



    progress = ((total_visited / trailsize)*100);
    return progress;
  }

  int Scheduler::getTarget() const
  {
    return target;
  }

  double Scheduler::getTargetLat() const
  {
    return trailcopy.getLat(target);
  }

  double Scheduler::getTargetLon() const
  {
    return trailcopy.getLon(target);
  }

  double Scheduler::getTargetOrient() const
  {
    return trailcopy.getOrient(target);
  }

  bool Scheduler::getDirection() const
  {
    return reversed;
  }

  bool Scheduler::getPaused() const
  {
    if(state == PAUSED)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // TODO A corrective mechanism or notification would make this more robust.
  bool Scheduler::metTarget(Node& n)
  {
    bool targetmet = false;
    
	/*
     * When the node reports that the robot's achieved its goal, a verification
	 * is done to ensure the robot is indeed in the approximate area of the
	 * intended waypoint, measuring the robot against a threshold position and
	 * orientation.
	 *
	 * Without verification, the state machine would simply traverse from
	 * GOALSENT to GOALACHIEVED as it would be reading the old node report for
	 * future targets.
     */
	if(n.goalAchieved())
	{
	  if(std::abs(n.getRobotYaw() - trailcopy.getOrient(target)) < 
	    target_threshold_yaw && sat_goal::distanceBetween(n.getRobotLat(),
		n.getRobotLon(), trailcopy.getLat(target), trailcopy.getLon(target)) <
		target_threshold_distance_metres)
      {
        targetmet = true;
      }
	}

	// Mark the waypoint as visited and update to a new target.
	if(targetmet)
	{
	  trailcopy.visit(target);

	  if(!reversed)
	  {
	    target++;
	  }
	  else
	  {
	    target--;
      }
	}

    return targetmet;
  }

  void Scheduler::sendTargetGoal(Node &n) const
  {
    double lat = trailcopy.getLat(target);
    double lon = trailcopy.getLon(target);
    double orient = trailcopy.getOrient(target);
    n.sendGoal(lat, lon, orient);
  }

  // a finite-state machine for processing active mission behaviour
  int Scheduler::process(Node &n)
  {
    switch(state) {
      case BEGIN:
        sendTargetGoal(n);
        state = GOALSENT;
        break;
      case GOALSENT:
        if(metTarget(n))
        {
          state = GOALACHIEVED;
        }
        break;
      case GOALACHIEVED:
        if(target > (trailcopy.getSize() - 1)
           && trailcopy.hasVisited((trailcopy.getSize() - 1))
           || (reversed && target == -1))
        {
          prev_state = FINISHED;
          state = STOPPING_ROBOT;
        } else {
          sendTargetGoal(n);
          state = GOALSENT;
        }
        break;
      case STOPPING_ROBOT:
        if(n.isMoving())
        {
          n.cancelGoal();
        }
        else
        {
          state = prev_state;
        }
      default:
        break;
    }

    return static_cast<int>(state);
  }

  Scheduler::SchedulerState Scheduler::getState() {
    return state;
  }
}
