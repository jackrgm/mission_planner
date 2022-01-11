#ifndef RECORDER_H
#define RECORDER_H

#include "mission_planner/node.h"
#include "mission_planner/trail.h"
#include "mission_planner/waypoint.h"

namespace mission_planner {
  // used for automatically capturing waypoints for a navigation trail
  class Recorder
  {
    public:
      Recorder();
      ~Recorder();

      void capture(const Node &n, Trail &t);
      void capture(int index, const Node &n, Trail &t);
      bool autoCapture(const Node &n, Trail &t);
      void reset();

    private:
      /*
       * A threshold of the robot turning is used as the main condition for
       * deciding whether a new waypoint should be automatically added to the
       * trail.
       *
       * If the robot has turned with a difference of -1 or +1 yaw, the
       * waypoint is added, otherwise the recorder carries on listening until the
       * threshold has been reached.
       *
       * To achieve this, two yaw readings are taken and the difference between
       * them is used for the condition check. The origin is either the first yaw
       * measured at runtime or it's a yaw taken after meeting a threshold. The
       * input yaw is used as the most current reading - if it meets the threshold
       * it becomes the origin yaw, otherwise it's practically discarded and
       * replaced by the most current reading the next time a condition check is
       * made.
       */
      double origin_yaw = 0;
      double input_yaw = 0;
      double threshold = 0.6;

      bool first_call = true;
  };
}
#endif
