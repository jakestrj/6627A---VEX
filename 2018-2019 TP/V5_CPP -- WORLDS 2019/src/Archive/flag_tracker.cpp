#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace c;
using namespace pros;
#include <cmath>

namespace flag_tracker {


  static const Flag FLAG_NONE = {false};


  // config
  #define SENSOR_ANGLE_RAD SENSOR_ANGLE * PI / 180


  // vision sensor
  Vision vision_sensor(10);


  // color codes
  vision_color_code_t code_red = vision_sensor.create_color_code(FLAG_SIG_RED, FLAG_SIG_GREEN);
  vision_color_code_t code_blue = vision_sensor.create_color_code(FLAG_SIG_GREEN, FLAG_SIG_BLUE);
  vision_color_code_t code_neutral = vision_sensor.create_color_code(FLAG_SIG_GREEN, FLAG_SIG_BLUE, FLAG_SIG_RED, FLAG_SIG_GREEN);


  // arrays of flags and flagpoles
  int flag_count = 0;
  Flag flags[MAX_FLAGS];
  Flagpole flagpoles[MAX_FLAGPOLES];


  // shoot recommendations
  bool in_range_double_shot = false;
  bool in_range_near_shot_mid = false;
  bool in_range_far_shot_mid = false;
  bool in_range_near_shot_top = false;
  bool in_range_far_shot_top = false;


  // calculate distance from vision y coordinate
  float calc_flag_dist(float height, float center_y) {
      return sin(SENSOR_ANGLE_RAD) * (height - SENSOR_HEIGHT) * DISTANCE_SCALAR / (VISION_IMAGE_WIDTH/2 - center_y); // UNTESTED CALCULATION
  }


  float get_pole_location(Flag flag) {
    if (flag.color == color_red) return flag.vision_x;
    if (flag.color == color_neutral) return flag.vision_x + flag.width/2; 
    return flag.vision_x + flag.width;
  }


  // create flag object
  Flag create_flag(float vision_x, float vision_y, float width, float height, Color color) {
    Flag flag;
    flag.identified = true;
    flag.vision_x = vision_x;
    flag.vision_y = vision_y;
    flag.width = width;
    flag.height = height;
    flag.color = color;
    flag.vision_pole_x = get_pole_location(flag);
    return flag;
  }


  // create flagpole object
  Flagpole create_flagpole(Flag flag_top, Flag flag_mid, Flag flag_btm) {
    Flagpole pole;

    if (flag_top.identified || flag_mid.identified || flag_btm.identified) {
      pole.identified = true;

      // calculate distance
      float dist_btm = (flag_btm.identified) ? calc_flag_dist(FLAG_HEIGHT_BTM, flag_btm.vision_y) : 0;
      float dist_mid = (flag_mid.identified) ? calc_flag_dist(FLAG_HEIGHT_MID, flag_mid.vision_y) : 0;
      float dist_top = (flag_top.identified) ? calc_flag_dist(FLAG_HEIGHT_TOP, flag_top.vision_y) : 0;
      int identified_count = flag_btm.identified + flag_mid.identified + flag_top.identified;
      pole.robot_dist = (dist_btm + dist_mid + dist_top) / identified_count;

      pole.robot_x = 1 * flag_btm.vision_pole_x / pole.robot_dist; // UNTESTED CALCULATION
      pole.flag_top = flag_top;
      pole.flag_mid = flag_mid;
      pole.flag_btm = flag_btm;
    }

    return pole;
  }


  // upate flag list
  void update_flags() {

    // "remove" all previously stored flags
    for (int i = 0; i < MAX_FLAGS; i++) {
      flags[i].identified = false;
    }

    // find new flags
    vision_object_s_t flags_red_raw[MAX_FLAGS];
    vision_object_s_t flags_blue_raw[MAX_FLAGS];
    vision_object_s_t flags_neutral_raw[MAX_FLAGS];
    int flag_count_neutral = vision_sensor.read_by_code(0, code_neutral, MAX_FLAGS, flags_neutral_raw);
    if (flag_count_neutral == PROS_ERR) flag_count_neutral = 0;
    int flag_count_red = vision_sensor.read_by_code(0, code_red, MAX_FLAGS - flag_count_neutral, flags_red_raw);
    if (flag_count_red == PROS_ERR) flag_count_red = 0;
    int flag_count_blue = vision_sensor.read_by_code(0, code_blue, MAX_FLAGS  - flag_count_neutral - flag_count_neutral, flags_blue_raw);
    if (flag_count_blue == PROS_ERR) flag_count_blue = 0;

    int flag_count_individual = flag_count_red + flag_count_blue;
    flag_count = flag_count_individual + flag_count_neutral;

    if (flag_count != PROS_ERR && flag_count > 0) {

      // convert to Flag structs
      Flag flags_unsorted_individual[flag_count_individual];
      Flag flags_unsorted_neutral[flag_count_neutral];
      for (int i = 0; i < flag_count_red; i++) {
        flags_unsorted_individual[i] = create_flag(flags_red_raw[i].left_coord, flags_red_raw[i].top_coord, flags_red_raw[i].width, flags_red_raw[i].height, color_red);
      }
      for (int i = 0; i < flag_count_blue; i++) {
        flags_unsorted_individual[i] = create_flag(flags_blue_raw[i].left_coord, flags_blue_raw[i].top_coord, flags_blue_raw[i].width, flags_blue_raw[i].height, color_blue);
      }
      for (int i = 0; i < flag_count_neutral; i++) {
        flags_unsorted_neutral[i] = create_flag(flags_neutral_raw[i].left_coord, flags_neutral_raw[i].top_coord, flags_neutral_raw[i].width, flags_neutral_raw[i].height, color_neutral);
      }

      // remove duplicates
      for (int i = 0; i < flag_count_neutral; i++) {
        for (int j = 0; j < flag_count_red; j++) {
          if (fabs(flags_unsorted_neutral[i].vision_pole_x - flags_unsorted_individual[j].vision_pole_x) <= SAME_POLE_BUFFER) {
            flags_unsorted_individual[j].identified = false;
            flag_count--;
          }
        }
      }

      // combine individual and neutral flags into one array
      Flag flags_unsorted[flag_count];
      int sort_index = 0;
      for (int i = 0; i < flag_count_individual; i++) {
        if (flags_unsorted_individual[i].identified) {
          flags_unsorted[sort_index] = flags_unsorted_individual[i];
          sort_index++;
        }
      }
      for (int i = 0; i < flag_count_neutral; i++) {
        if (flags_unsorted_neutral[i].identified) {
          flags_unsorted[sort_index] = flags_unsorted_neutral[i];
          sort_index++;
        }
      }

      // sort by x coordinate and store in flags array
      for (int i = 0; i < flag_count; i++) {
        int min_index = 0;
        float min_x = flags_unsorted[0].vision_x;

        for (int j = 0; j < flag_count; j++) {
          if (flags_unsorted[min_index].identified == false || (flags_unsorted[j].identified && flags_unsorted[j].vision_x > min_x)) {
            min_index = j;
            min_x = flags_unsorted[j].vision_x;
          }
        }

        flags[i] = flags_unsorted[min_index];
        flags_unsorted[min_index].identified = false;
      }
    }
  }


  // update flagpole list
  void update_flagpoles() {
    
    // "remove" all previously stored flagpoles
    for (int i = 0; i < MAX_FLAGPOLES; i++) {
      flagpoles[i].identified = false;
    }
    int pole_index = 0;

    // make copy of flags array
    Flag flags_cpy[flag_count];
    for (int i = 0; i < flag_count; i++) flags_cpy[i] = flags[i];

    // loop through and find flags with same flagpole
    for (int i = 0; i < flag_count; i++) {
      if (pole_index < MAX_FLAGPOLES && flags_cpy[i].identified) {
        flags_cpy[i].identified = false;

        // calculate pole location
        float pole_location = flags_cpy[i].vision_pole_x;

        // create small array to store flags on pole
        Flag pole[3];
        pole[0] = flags_cpy[i];
        int pole_flags = 1;

        // find other flags on same pole
        for (int j = i; j < flag_count; j++) {
          if (pole_flags < 3 && fabs(flags_cpy[j].vision_pole_x - pole_location) <= SAME_POLE_BUFFER) {
            pole[pole_flags] = flags_cpy[j];
            pole_flags++;
            flags_cpy[j].identified = false;
          }
        }

        // sort flags (lowest -> highest)
        Flag pole_sorted[pole_flags];
        for (int j = 0; j < pole_flags; j++) {
          int min_index = 0;
          float min_height = pole[0].vision_y;

          for (int k = 0; k < pole_flags; k++) {
            if (pole[min_index].identified == false || (pole[k].identified && pole[k].vision_y < min_height)) {
              min_index = k;
              min_height = pole[k].vision_y;
            }
          }

          pole_sorted[j] = pole[min_index];
          pole[min_index].identified = false;
        }


        // put flags onto flagpole object
        flagpoles[pole_index] = create_flagpole(
          (pole_flags >= 3) ? pole_sorted[2] : FLAG_NONE,
          (pole_flags >= 2) ? pole_sorted[1] : FLAG_NONE,
          (pole_flags >= 1) ? pole_sorted[0] : FLAG_NONE
        );

        pole_index++;
      }
    }
  }
  

  // update shoot recommendations
  void update_recommendations() {

    // default all to false
    in_range_double_shot = false;
    in_range_near_shot_mid = false;
    in_range_far_shot_mid = false;
    in_range_near_shot_top = false;
    in_range_far_shot_top = false;

    // loop through all flagpoles and set recommendations to true if in range
    for (int i = 0; i < MAX_FLAGPOLES; i++) {
      if (flagpoles[i].identified && fabs(flagpoles[i].robot_x) < 10) {
        if (fabs(flagpoles[i].robot_dist - fire_dist_double) < 5) in_range_double_shot = true;
        if (fabs(flagpoles[i].robot_dist - fire_dist_single_near_mid) < 10) in_range_near_shot_mid = true;
        if (fabs(flagpoles[i].robot_dist - fire_dist_single_far_mid) < 10) in_range_far_shot_mid = true;
        if (fabs(flagpoles[i].robot_dist - fire_dist_single_near_top) < 5) in_range_near_shot_top = true;
        if (fabs(flagpoles[i].robot_dist - fire_dist_single_far_top) < 5) in_range_far_shot_top = true;
      }
    }
  }
}
