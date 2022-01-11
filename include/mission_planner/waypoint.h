#ifndef WAYPOINT_H
#define WAYPOINT_H

namespace mission_planner {
  struct Waypoint
  {
    double latitude;
    double longitude;
    double orientation;
    bool visited = false;
  };
}
#endif
