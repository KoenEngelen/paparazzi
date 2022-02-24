#include "subsystems/navigation/waypoints.h"


void SetWaypointsToSquare(uint8_t wp_0, uint8_t wp_1, uint8_t wp_2, uint8_t wp_3){
  // initialize coordinate arrays for waypoints
  float x_coords[4] = {0};
  float y_coords[4] = {0};
  // get X coordinates of all waypoints
  x_coords[0] = waypoint_get_x(wp_0);
  x_coords[1] = waypoint_get_x(wp_1);
  x_coords[2] = waypoint_get_x(wp_2);
  x_coords[3] = waypoint_get_x(wp_3);
  // get Y coordinates of all waypoints
  y_coords[0] = waypoint_get_y(wp_0);
  y_coords[1] = waypoint_get_y(wp_1);
  y_coords[2] = waypoint_get_y(wp_2);
  y_coords[3] = waypoint_get_y(wp_3);
  // initialize vectors inbetween waypoints
  float delta_0[2] = {0};
  float delta_1[2] = {0};
  // get difference between wp_0 and wp_1
  delta_0[0] = x_coords[1] - x_coords[0];
  delta_0[1] = y_coords[1] - y_coords[0];
  // get difference between wp_1 and wp_2
  delta_1[0] = x_coords[2] - x_coords[1];
  delta_1[1] = y_coords[2] - y_coords[1];
  // get dot product between deltas (for perpendicular vector)
  float dot = delta_0[0] * delta_1[0] + delta_0[1] * delta_1[1];
  // get perpendicular component of delta_1
  delta_1[0] = (delta_1[0] - dot * delta_0[0]);
  delta_1[1] = (delta_1[1] - dot * delta_0[1]);
  // compute magnetudes of both vectors
  float mag_0 = sqrt(delta_0[0] * delta_0[0] + delta_0[1] * delta_0[1]);
  float mag_1 = sqrt(delta_1[0] * delta_1[0] + delta_1[1] * delta_1[1]);
  // set magnitude of delta_2 to the magnitude of delta_1
  delta_1[0] = delta_1[0]*mag_0/mag_1;
  delta_1[1] = delta_1[1]*mag_0/mag_1;
  // update all waypoints (wp_0 and wp_1 remain the same)
  struct EnuCoor_f pos_2;
  struct EnuCoor_f pos_3;
  pos_2.x = x_coords[1] + delta_0[0];
  pos_2.y = y_coords[1] + delta_0[1];
  pos_3.x = x_coords[0] + delta_0[0];
  pos_3.y = y_coords[0] + delta_0[1];
  waypoint_set_enu(wp_2, &pos_2);
  waypoint_set_enu(wp_3, &pos_3);
}

void SnapWaypointsToTrajectory(uint8_t wp_0, uint8_t wp_1, float length){
  // initialize coordinate arrays for waypoints
  float x_coords[2] = {0};
  float y_coords[2] = {0};
  // get X coordinates of all waypoints
  x_coords[0] = waypoint_get_x(wp_0);
  x_coords[1] = waypoint_get_x(wp_1);
  // get Y coordinates of all waypoints
  y_coords[0] = waypoint_get_y(wp_0);
  y_coords[1] = waypoint_get_y(wp_1);
  // initialize vector inbetween waypoints
  float delta[2] = {0};
  // get difference between wp_0 and wp_1
  delta[0] = x_coords[1] - x_coords[0];
  delta[1] = y_coords[1] - y_coords[0];
  // compute magnetudes of vector
  float mag = sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
  // set magnitude of delta to length of trajectory
  delta[0] = delta[0]*length/mag;
  delta[1] = delta[1]*length/mag;
  // update waypoint (wp_0 remains the same)
  struct EnuCoor_f pos;
  pos.x = x_coords[0] + delta[0];
  pos.y = y_coords[0] + delta[1];
  waypoint_set_enu(wp_1, &pos);
}