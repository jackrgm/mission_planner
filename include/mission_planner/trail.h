#ifndef TRAIL_H
#define TRAIL_H

#include <vector>
#include "mission_planner/waypoint.h"

namespace mission_planner {
  // TODO organise ordering etc
  class Trail
  {
  public:
    // TODO take name, maybe some other meta info like date and time of edit?
    Trail();
    ~Trail();

    void add(double lat, double lon, double orient);
    void clear();
    void edit(int index, double lat, double lon, double orient);
    double getLat(int index) const;
    double getLon(int index) const;
    double getOrient(int index) const;
    int getSize() const;
    void insertAfter(int index, double lat, double lon, double orient);
    int load(std::string dir);
    void moveAfter(int selection, int position);
    void remove(int index);
    int save(std::string dir);
    int visit(int index);
    bool hasVisited(int index) const;

    // 0 = towards head, 1 = towards tail
    void shift(int index, bool dir);
  private:
    std::vector<Waypoint> waypoints;

    bool indexCheck(int index) const;
  };
}
#endif
