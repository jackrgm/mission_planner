#include <gtest/gtest.h>
#include "mission_planner/trail.h"

using namespace mission_planner;

TEST(Trail, insertAfter)
{
  Trail t;

  // Add a first waypoint (lat, lon, and yaw).
  t.add(111.111, 111.111, 1.1111);
  EXPECT_EQ(1, t.getSize());
  EXPECT_EQ(111.111, t.getLat(0));

  // Validate insertion after the first waypoint.
  t.insertAfter(0, 222.222, 222.222, 2.2222);
  EXPECT_EQ(2, t.getSize());
  EXPECT_EQ(111.111, t.getLat(0));
  EXPECT_EQ(222.222, t.getLat(1));

  // Validate insertion before first waypoint (insert at beginning).
  t.insertAfter(-1, 0, 0, 0);
  EXPECT_EQ(3, t.getSize());
  EXPECT_EQ(0, t.getLat(0));
  EXPECT_EQ(111.111, t.getLat(1));
  EXPECT_EQ(222.222, t.getLat(2));
}

TEST(Trail, shifting) {
  double latitude = 111.111111;
  double longitude = 111.111111;
  double orientation_yaw = 1.1111;

  Trail t;

  for(int i = 0; i < 4; i++) {
    t.add(latitude, longitude, orientation_yaw);
  }

  EXPECT_EQ(4, t.getSize());

  // set a different value as it's the test dummy to be shifted in the trail
  double latitude2 = 222.222222;
  double longitude2 = 222.222222;
  double orientation_yaw2 = 2.2222;

  t.add(latitude2, longitude2, orientation_yaw2);

  /*
   * shifting up (towards the head/beginning):
   * shift index 5 towards the head twice, so it should be at position 3
   *
   * 0
   * 1
   * 2 <--- after second shift call (should be here when we evaluate)
   * 3 <-- after first shift call
   * 4 <- start
   */
  t.shift(4, true);
  t.shift(3, true);
  EXPECT_EQ(latitude2, t.getLat(2));
  EXPECT_EQ(longitude2, t.getLon(2));
  EXPECT_EQ(orientation_yaw2, t.getOrient(2));

  /*
   * shifting down (towards the tail/end):
   * shift index 3 towards the tail twice, so it should be at position 5
   *
   * 0
   * 1
   * 2 <- start (from earlier shifting upwards)
   * 3 <-- after first shift call
   * 4 <--- after second shift call
   */
  t.shift(2, false);
  t.shift(3, false);

  EXPECT_EQ(latitude2, t.getLat(4));
  EXPECT_EQ(longitude2, t.getLon(4));
  EXPECT_EQ(orientation_yaw2, t.getOrient(4));
}

TEST(Trail, adding) {
  double latitude1 = 49.80000000853;
  double longitude1 = 8.090000021423;
  double orientation_yaw1 = -2.30843;

  double latitude2 = 1.10000000001;
  double longitude2 = 2.200000000002;
  double orientation_yaw2 = 3.00000;

  Trail t;

  EXPECT_EQ(0, t.getSize());
  t.add(latitude1, longitude1, orientation_yaw1);
  EXPECT_EQ(1, t.getSize());
  EXPECT_EQ(latitude1, t.getLat(0));
  EXPECT_EQ(longitude1, t.getLon(0));
  EXPECT_EQ(orientation_yaw1, t.getOrient(0));

  t.add(latitude2, longitude2, orientation_yaw2);
  EXPECT_EQ(2, t.getSize());
  EXPECT_EQ(latitude2, t.getLat(1));
  EXPECT_EQ(longitude2, t.getLon(1));
  EXPECT_EQ(orientation_yaw2, t.getOrient(1));
}

TEST(Trail, removing) {
  double latitude = 123.4523426;
  double longitude = 1234.234234;
  double orientation_yaw = 1.2134;

  Trail t;

  EXPECT_EQ(0, t.getSize());
  t.add(latitude, longitude, orientation_yaw);
  EXPECT_EQ(1, t.getSize());
  t.remove(0);
  EXPECT_EQ(0, t.getSize());
}

TEST(Trail, editing) {
  double latitude1 = 123.456;
  double longitude1 = 789.101112;
  double orientation_yaw1 = 0.123;

  double latitude2 = 456.789;
  double longitude2 = 101112.131415;
  double orientation_yaw2 = -2.310;

  Trail t;

  t.add(latitude1, longitude1, orientation_yaw1);
  EXPECT_EQ(latitude1, t.getLat(0));
  EXPECT_EQ(longitude1, t.getLon(0));
  EXPECT_EQ(orientation_yaw1, t.getOrient(0));

  t.edit(0, latitude2, longitude2, orientation_yaw2);
  EXPECT_EQ(latitude2, t.getLat(0));
  EXPECT_EQ(longitude2, t.getLon(0));
  EXPECT_EQ(orientation_yaw2, t.getOrient(0));
}

TEST(Trail, clearing) {
  double latitude = 123.456;
  double longitude = 123.456;
  double orientation_yaw = -2.321;

  Trail t;

  for(int i = 0; i < 10; i++) {
    t.add(latitude, longitude, orientation_yaw);
  }

  EXPECT_EQ(10, t.getSize());
  t.clear();
  EXPECT_EQ(0, t.getSize());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
