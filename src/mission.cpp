#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <unistd.h>
#include "mission_planner/node.h"
#include "mission_planner/recorder.h"
#include "mission_planner/scheduler.h"
#include "mission_planner/trail.h"

using namespace mission_planner;

namespace mission {
  void help() {
    std::cout << "Available commands: \n\n";

    std::cout << "goal-send" << "\t Send a single waypoint to the robot\n";
    std::cout << "goal-stop" << "\t Cancel the current waypoint\n";
    std::cout << "\n";
    std::cout << "trail-add" << "\t Add new waypoint to trail using specified coordinates (direct input)\n";
    std::cout << "trail-shift-u" << "\t Move a waypoint up in the trail list\n";
    std::cout << "trail-shift-d" << "\t Move a waypoint down in the trail list\n";
    std::cout << "trail-move" << "\t Move a waypoint to another position in the trail list\n";
    std::cout << "trail-remove" << "\t Remove a waypoint from the trail\n";
    std::cout << "trail-clear" << "\t Wipe the trail\n";
    std::cout << "\n";
    std::cout << "mission-start" << "\t Start a mission, using the current working trail as input\n";
    std::cout << "mission-abort" << "\t Stop a mission completely\n";
    std::cout << "mission-pause" << "\t Pause or continue the current mission\n";
    std::cout << "mission-reverse" << "\t Reverse the robot's navigation through the trail (or vice versa)\n";
    std::cout << "\n";
    std::cout << "show-mission" << "\t Show a 2D map view of the current mission\n";
    std::cout << "show-pos" << "\t Show the robot's current position.\n";
    std::cout << "show-progress" << "\t Show progress \% of the mission\n";
    std::cout << "show-target" << "\t Show the robot's current target waypoint\n";
    std::cout << "show-trail" << "\t Show the current contents of the trail\n";
    std::cout << "show-all" << "\t Show all information about the mission\n";
    std::cout << "\n";
    std::cout << "capture" << "\t\t Add new waypoint to trail using robot's current location\n";
    std::cout << "capture-a" << "\t Start automatic trail capture\n";
  }

  double promptLatLon(bool type)
  {
    double input;
    std::string lat_or_lon;

    if (!type) {
      lat_or_lon = "latitude";
    }
    else
    {
      lat_or_lon = "longitude";
    }

    std::cout << "Enter " << lat_or_lon << ": ";
    std::cin >> input;

    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return input;
  }

  // TODO make a generalised 'prompt' function with a pointer param + tryagains
  double promptOrient()
  {
    double input = 0;

    std::cout << "Enter orientation (yaw): ";
    std::cin >> input;

    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return input;
  }

  // TODO contents are being rounded/shortened (e.g. 49.9 not 49.9000042123 etc)
  /* print the list, format example:
   *
   * Trail contents:
   *
   * 1. 52.414311, -4.081685
   * 2. 52.430200, -4.082343
   */
  void show_trail(const Trail &t)
  {
    if (t.getSize() == 0)
    {
      std::cout << "Trail is empty\n";
      return;
    }

    std::cout << "Trail contents:\n";

    std::cout.precision(12);

    for (int i = 0; i < t.getSize(); i++)
    {
      std::cout << "\t";

      // the index
      std::cout << i+1 << ": ";

      // the index's latitude
      std::cout << t.getLat(i) << ", ";

      // the index's longitude
      std::cout << t.getLon(i) << ", ";

      // the index's orientation
      std::cout << t.getOrient(i) << "\n";
    }
  }

  void goal_send(Node &n)
  {
    double lat, lon, orient;

    lat = promptLatLon(0);
    lon = promptLatLon(1);
    orient = promptOrient();

    n.sendGoal(lat, lon, orient);
    return;
  }

  void goal_cancel(Node &n, Scheduler &s)
  {
    // TODO is this prone to error? (e.g. does target -1 always mean inactive?)
    if(s.getTarget() > -1 && !s.getPaused())
    {
      std::cout << "Note: a mission is active, pausing mission instead\n";
      s.pause(n);
    }
    else
    {
      // TODO check for active goal first; no need to cancel a non-goal
      n.cancelGoal();
    }
  }

  void trail_add(Trail &t)
  {
    int index = -2;
    double lat, lon, orient;
    std::string input = "";

    if(t.getSize() > 0)
    {
      // TODO Make a prompt_index function.
      show_trail(t);
      std::cout << "Type an index number to insert AFTER "
                   "(0 to insert at beginning, default: last index): ";

      if(!std::getline(std::cin, input))
      {
        std::cout << "Error: problem reading input, defaulting to last "
          "index\n";
      }
      else if(input == "")
      {
        // Default input (no answer): add to end of trail.
      }
      else
      {
        std::istringstream istrings(input);
        istrings >> index;

        if(!istrings)
        {
          std::cout << "Note: bad input, defaulting to last index\n";
          index = -2;
        }
        // TODO Conversion to 0-index should ideally be done once (if possible).
        else if (istrings.get() != EOF )
        {
          // Error: unexpected garbage at end of line...
          index--;
        }
        else
        {
          // Convert to 0-indexing.
          index--;

          if (index < -1)
          {
            std::cout << "Note: given index (" << (index + 1) << ") out of "
              "bounds, defaulting to inserting at beginning\n";
            index = -1;
          }
          else if(index > (t.getSize() - 1))
          {
            std::cout << "Note: given index (" << index << ") out of bounds, "
              "defaulting to adding as last index\n";
            index = -2;
          }
        }
      }
    }

    lat = promptLatLon(0);
    lon = promptLatLon(1);
    orient = promptOrient();

    if(t.getSize() > 0 && index > -2 && index < t.getSize())
    {
      t.insertAfter(index, lat, lon, orient);
      return;
    }
    else
    {
      t.add(lat, lon, orient);
    }
  }

  void trail_remove(Trail &t)
  {
    int index = -1;
    bool inputfail = false;

    if(t.getSize() > 0)
    {
      show_trail(t);
      std::cout << "Type an index number to remove: ";
      std::cin >> index;

      // convert to 0-indexing
      --index;

      inputfail = std::cin.fail();
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      if(inputfail)
      {
        std::cout << "Error: bad input (not a number), no operations done\n";
        return;
      }
      else if(index < 1 | index > t.getSize())
      {
        std::cout << "Error: No such index, no operations done\n";
        return;
      }
      else
      {
        t.remove(index);
      }
    }
    else
    {
      std::cout << "Trail is empty, nothing to remove\n";
    }
  }

  // TODO finish (only prints, need to then take input and update model)
  void trail_shift(bool dir, Trail &t)
  {
    int index = 0;
    std::string dirlabel;
    bool direction;
    bool inputfail = false;

    if(t.getSize() < 2)
    {
      std::cout << "Error: not enough waypoints in the trail to shift\n";
      return;
    }

    if(dir)
    {
      dirlabel = "up";
      direction = true;
    }
    else
    {
      dirlabel = "down";
      direction = false;
    }

    show_trail(t);

    std::cout << "Select an index to shift " << dirlabel << ": ";
    std::cin >> index;

    // convert to 0-indexing
    --index;

    inputfail = std::cin.fail();
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if(inputfail)
    {
      std::cout << "Error: bad input (not a number), no operations done\n";
      return;
    }
    else if(index < 1 | index > t.getSize())
    {
      std::cout << "Error: No such index, no operations done\n";
      return;
    }
    else
    {
      t.shift(index, direction);
    }
  }

  // TODO implement
  void trail_move(Trail &t)
  {
    int target, dest;
    show_trail(t);

    // target = prompt_index(t, "Select an index to move:");
    /*
     * dest = prompt_index(t, "Select an index to insert AFTER (default: "
     * "end of trail";
     */

    t.insertAfter(dest, t.getLat(target), t.getLon(target), t.getOrient(target));
    t.remove(target);
  }

  void trail_clear(Trail &t)
  {
    t.clear();
  }

  void mission_start(Trail t, Scheduler &s, Node &n)
  {
    if (t.getSize() < 1) {
      std::cout << "Error: can't start a mission with an empty trail\n";
      return;
    }

    Scheduler tmp(t, n);
    s = tmp;
  }

  void mission_abort(Scheduler &s, Node &n)
  {
    s.abort(n);
  }

  void mission_pause(Scheduler &s, Node &n)
  {
    s.pause(n);
  }

  void mission_reverse(Scheduler &s, Node &n)
  {
    s.reverse(n);
  }

  // TODO implement (waiting on scheduler implementation)
  void show_mission(const Node &n, const Scheduler &s, const Trail &t)
  {
    // TODO get mission data
    /*
    s.getLastVisited();
    s.getProgress();
    s.getTarget();
    s.getDirection();
    */

    // TODO get robot data
    /*
    n.getRobotLat();
    n.getRobotLon();
    n.getRobotYaw();
    */

    // TODO generate some text-based graphic of the mission
    std::cout << "Displaying mission overview graphic not yet implemented\n";
  }

  void show_pos(const Node &n)
  {
    std::cout << "Robot's current position: " << n.getRobotLat() << ", "
              << n.getRobotLon() << n.getRobotYaw();
  }

  void show_progress(Scheduler &s)
  {
    std::cout << "Progress:" << s.getProgress() << "\n";
  }

  void show_target(const Scheduler &s, const Trail &t)
  {
    std::cout << "#" << s.getTarget() << ": " << t.getLat(s.getTarget()) << ", "
              << t.getLon(s.getTarget()) << "\n";
  }

  // TODO implement
  void show_all(Scheduler &s)
  {
    std::cout << "Displaying all mission details not yet implemented\n";
  }

  // TODO implement
  void cap_auto(Recorder &r, Trail &t, const Node &n)
  {
    std::cout << "capauto\n";
  }

  void cap_man(Recorder &r, Trail &t, const Node &n)
  {
    r.capture((t.getSize() - 1), n, t);
  }

  void print_intro()
  {
    std::cout << "Mission v0.0.0\n";
    std::cout << "Type \'help\' for a list of available commands\n";
  }
}

using namespace mission;

// TODO add a bool promptIndex(index i) func
/*
 * Mission is a simple command-line client that uses the mission_planner library
 * and primarily exists as a quick testing and demo space.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission");
  Node node;
  Recorder recorder;
  std::string in;
  bool quit = false;

  print_intro();

  Scheduler scheduler;
  Trail trail;

  auto th = std::thread([&]() {
    while(!quit) {
      node.update();
      scheduler.process(node);
      usleep(1000);
    }
  });

  // TODO create a command-line parser class, or function, to abstract this
  while(1)
  {
    std::cout << "> ";
    std::cin >> in;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if(in == "help")
    {
      help();
    }
    else if (in == "goal-send")
    {
      goal_send(node);
    }
    else if (in == "goal-stop")
    {
      goal_cancel(node, scheduler);
    }
    else if (in == "trail-add")
    {
      trail_add(trail);
    }
    else if (in == "trail-remove")
    {
      trail_remove(trail);
    }
    else if (in == "trail-clear")
    {
      trail_clear(trail);
    }
    else if (in == "trail-shift-u")
    {
      trail_shift(true, trail);
    }
    else if (in == "trail-shift-d")
    {
      trail_shift(false, trail);
    }
    else if (in == "trail-move")
    {
      trail_move(trail);
    }
    else if (in == "mission-start")
    {
      mission_start(trail, scheduler, node);
    }
    else if (in == "mission-abort")
    {
      mission_abort(scheduler, node);
    }
    else if (in == "mission-pause")
    {
      mission_pause(scheduler, node);
    }
    else if (in == "mission-reverse")
    {
      mission_reverse(scheduler, node);
    }
    else if (in == "show-mission")
    {
      show_mission(node, scheduler, trail);
    }
    else if (in == "show-pos")
    {
      show_pos(node);
    }
    else if (in == "show-progress")
    {
      show_progress(scheduler);
    }
    else if (in == "show-target")
    {
      show_target(scheduler, trail);
    }
    else if (in == "show-trail")
    {
      show_trail(trail);
    }
    else if (in == "show-all")
    {
      show_all(scheduler);
    }
    else if (in == "capture")
    {
      cap_man(recorder, trail, node);
    }
    else if (in == "capture-a")
    {
      cap_auto(recorder, trail, node);
    }
    else if (in == "state")
    {
      std::cout << "state: " << scheduler.getState();
    }
    else if (in == "quit" | in == "exit")
    {
      node.cancelGoal();
      quit = true;
      th.join();
      break;
    }
    else
    {
      std::cout << "Command not recognised (type \'help\' to see options)\n";
    }
  }
}
