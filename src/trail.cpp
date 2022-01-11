#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "mission_planner/trail.h"

namespace mission_planner {
  Trail::Trail()
  {
  }

  Trail::~Trail()
  {
  }

  void Trail::add(double lat, double lon, double orient)
  {
    mission_planner::Waypoint w;
    w.latitude = lat;
    w.longitude = lon;
    w.orientation = orient;

    waypoints.push_back(w);
  }

  void Trail::clear()
  {
    waypoints.clear();
  }

  void Trail::edit(int index, double lat, double lon, double orient)
  {
    waypoints.at(index).latitude = lat;
    waypoints.at(index).longitude = lon;
    waypoints.at(index).orientation = orient;
  }

  // TODO does vector have any exception stuff built-in?
  bool Trail::indexCheck(int index) const
  {
    if(waypoints.empty())
    {
      // TODO throw empty exception
      return false;
    }
    else if(index < 0 || index > (getSize() - 1))
    {
      // TODO throw out of bounds exception
      return false;
    }
    else
    {
      return true;
    }
  }

  double Trail::getLat(int index) const
  {
    if(!indexCheck(index))
    {
      // TODO exception handling
      return 0.0;
    }
    else
    {
      return waypoints.at(index).latitude;
    }
  }

  double Trail::getLon(int index) const
  {
    if(!indexCheck(index))
    {
      // TODO exception handling
      return 0.0;
    }
    else
    {
      return waypoints.at(index).longitude;
    }
  }

  double Trail::getOrient(int index) const
  {
    if(!indexCheck(index))
    {
      // TODO exception handling
      return 0.0;
    }
    else
    {
      return waypoints.at(index).orientation;
    }
  }

  int Trail::getSize() const
  {
    return waypoints.size();
  }

  bool Trail::hasVisited(int index) const {
    if(!indexCheck(index))
    {
      // TODO exception handling
      return 0.0;
    }
    else
    {
      return waypoints.at(index).visited;
    }
  }

  void Trail::insertAfter(int index, double lat, double lon, double orient)
  {
    Waypoint w;
    w.latitude = lat;
    w.longitude = lon;
    w.orientation = orient;

    // TODO not trail's job for boundary enforcement, should throw etc instead
    if (index < -1 || index > (getSize() - 1))
    {
      add(lat, lon, orient);
      return;
    }

    waypoints.insert(waypoints.begin()+index+1, w);
  }

  int Trail::load(std::string dir)
  {
    clear();

    std::fstream file;
    file.open(dir, std::ios::in);
    double lat;
    double lon;
    double yaw;

    if(!file.is_open())
    {
      // TODO throw exception
      return 1;
    }
    else
    {
      std::string line;

      while(std::getline(file, line))
      {
        std::istringstream istrings(line);
        istrings >> lat >> lon >> yaw;
        add(lat, lon, yaw);
        istrings.str("");
        istrings.clear();
      }

      file.close();
      return 0;
    }

    return 1;
  }

  void Trail::moveAfter(int selection, int position)
  {
    // TODO if position < -1 (e.g. wants to moveAfter -2, into position -1) throw ex
    waypoints.insert(waypoints.begin()+position, waypoints.at(selection));
    waypoints.erase(waypoints.begin()+selection);
  }

  void Trail::remove(int index)
  {
    // TODO if trail empty or index out of bounds, throw exception(?)
    if(getSize() < 1 || index < 0 || index > (getSize() - 1))
    {
      return;
    }
    else
    {
      auto it = waypoints.begin();
      waypoints.erase(it+(index));
    }
  }

  // TODO add arguement to allow append, with ios::app mode, e.g. if file exists
    // e.g. std::string dir, bool overwrite
  int Trail::save(std::string dir)
  {
    std::fstream file;
    file.open(dir, std::ios::out);

    if(!file.is_open())
    {
      // TODO throw exception
      return 1;
    }
    else
    {
      for(int i = 0; i < getSize(); i++)
      {
        file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10) << getLat(i) << " " << getLon(i) << " " << getOrient(i) << "\n";
      }
      file.close();
    }
  }

  // TODO call exception instead of boundary enforcement
  void Trail::shift(int index, bool dir)
  {
    int swaptarget;
    Waypoint w;

    if (!indexCheck(index))
    {
      return;
    }
    else
    {
      // towards head
      if (dir)
      {
        swaptarget = index - 1;

        // boundary enforcement
        if(swaptarget < 0)
        {
          swaptarget = 0;
        }
      }
      // towards tail
      else
      {
        swaptarget = index + 1;

        // boundary enforcement
        if(swaptarget > (getSize() - 1))
        {
          swaptarget = getSize() - 1;
        }
      }
      std::iter_swap(waypoints.begin() + index, waypoints.begin() + swaptarget);
    }

    return;
  }

  int Trail::visit(int index) {
    if(index > -1 && index < getSize()) {
      waypoints.at(index).visited = true;

      return 0;
    }

    // error
    return 1;
  }
}
