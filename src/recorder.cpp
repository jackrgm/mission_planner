#include <iostream>
#include "mission_planner/recorder.h"

namespace mission_planner {
  Recorder::Recorder()
  {

  }

  Recorder::~Recorder()
  {

  }

  void Recorder::reset()
  {
    origin_yaw = 0;
    input_yaw = 0;
    first_call = true;
  }

  void Recorder::capture(const Node &n, Trail &t)
  {
    t.add(n.getRobotLat(), n.getRobotLon(), n.getRobotYaw());
  }

  void Recorder::capture(int index, const Node &n, Trail &t)
  {
    t.insertAfter(index, n.getRobotLat(), n.getRobotLon(), n.getRobotYaw());
  }

  // TODO also add a 'or' threshold to capture when robot's traveled x distance
    // use distanceBetween(latlon) from scheduler (maybe move it into its own class)
    //
  bool Recorder::autoCapture(const Node &n, Trail &t)
  {
    if(first_call)
    {
      origin_yaw = n.getRobotYaw();
      capture(n, t);
      first_call = false;
      return true;
    }
    else
    {
      input_yaw = n.getRobotYaw();

      if(input_yaw > (origin_yaw + threshold)
        || input_yaw < (origin_yaw - threshold))
      {
        capture(n, t);
        origin_yaw = input_yaw;
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}
