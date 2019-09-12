#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace c;
using namespace pros;

namespace cap_tracker {


  // config
  #define SENSOR_ANGLE_RAD SENSOR_ANGLE * PI / 180
  #define ZERO_DIST (SENSOR_ANGLE_RAD == 0) ? 0 : SENSOR_HEIGHT / tan(SENSOR_ANGLE_RAD)


  // vision sensor
  Vision vision_sensor(9);
  // array of caps
  int cap_count = 0;
  Cap caps[MAX_CAPS];


  // calculate distance from vision y coordinate
  float calc_cap_dist(float center_y) {
    return ZERO_DIST + DISTANCE_SCALAR * (VISION_IMAGE_HEIGHT/2 - center_y) / sin(SENSOR_ANGLE_RAD); // UNTESTED CALCULATION
  }


  // create cap object
  Cap create_cap(float vision_x, float vision_y, float width, float height, Color color) {
    Cap b;
    b.identified = true;
    b.vision_x = vision_x;
    b.vision_y = vision_y;
    b.robot_dist = calc_cap_dist(vision_y);
    b.robot_x = (VISION_IMAGE_WIDTH/2 - vision_x) / (VISION_IMAGE_HEIGHT - vision_y) * b.robot_dist; // UNTESTED CALCULATION
    b.color = color;
    return b;
  }


  // update cap list
  void update() {

    // "remove" all previously stored caps
    for (int i = 0; i < MAX_CAPS; i++) {
      caps[i].identified = false;
    }

    // find new caps
    vision_object_s_t caps_red_raw[MAX_CAPS];
    vision_object_s_t caps_blue_raw[MAX_CAPS];
    int cap_count_red = vision_sensor.read_by_sig(0, CAP_SIG_RED, MAX_CAPS, caps_red_raw);
    int cap_count_blue = vision_sensor.read_by_sig(0, CAP_SIG_BLUE, MAX_CAPS - ((cap_count_red == PROS_ERR) ? 0 : cap_count_red), caps_blue_raw);

    if (cap_count_red == PROS_ERR || cap_count_red < 1) cap_count_red = 0;
    if (cap_count_blue == PROS_ERR || cap_count_blue < 1) cap_count_blue = 0;
    cap_count = cap_count_red + cap_count_blue;

    if (cap_count != PROS_ERR && cap_count > 0) {

      // convert to Cap structs
      Cap caps_unsorted[cap_count];
      for (int i = 0; i < cap_count_red; i++) {
        caps_unsorted[i] = create_cap(caps_red_raw[i].left_coord, caps_red_raw[i].top_coord, caps_red_raw[i].width, caps_red_raw[i].height, color_red);
      }
      for (int i = 0; i < cap_count_blue; i++) {
        caps_unsorted[i] = create_cap(caps_blue_raw[i].left_coord, caps_blue_raw[i].top_coord, caps_blue_raw[i].width, caps_blue_raw[i].height, color_blue);
      }

      // sort by distance and store in caps array
      for (int i = 0; i < cap_count; i++) {
        int min_index = 0;
        float min_dist = caps_unsorted[0].robot_dist;

        for (int j = 0; j < cap_count; j++) {
          if (caps_unsorted[min_index].identified == false || (caps_unsorted[j].identified && caps_unsorted[j].robot_dist < min_dist)) {
            min_index = j;
            min_dist = caps_unsorted[j].robot_dist;
          }
        }

        caps[i] = caps_unsorted[min_index];
        caps_unsorted[min_index].identified = false;
      }
    }
  }
}
