/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "firmwares/rotorcraft/guidance/guidance_aerobatic.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in file_logger_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void file_logger_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "acc_x,acc_y,acc_z,");
  fprintf(file, "qi,qx,qy,qz,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "ref_pos_x,ref_pos_y,ref_pos_z,");
  fprintf(file, "ref_vel_x,ref_vel_y,ref_vel_z,");
  fprintf(file, "ref_acc_x,ref_acc_y,ref_acc_z,");
  fprintf(file, "ref_qi,ref_qx,ref_qy,ref_qz,");
  fprintf(file, "ref_rate_p,ref_rate_q,ref_rate_r,");
  fprintf(file, "ref_acc_p,ref_acc_q,ref_acc_r,");
  fprintf(file, "filt_acc_x,filt_acc_y,filt_acc_z,");
  fprintf(file, "filt_qi,filt_qx,filt_qy,filt_qz,");
  fprintf(file, "filt_rate_p,filt_rate_q,filt_rate_r,");
  fprintf(file, "filt_acc_p,filt_acc_q,filt_acc_r,");
  fprintf(file, "motor_mix_A1,motor_mix_A2,motor_mix_B1,motor_mix_B2,");
  fprintf(file, "ctrl_mode,ctrl_axes,");
#ifdef COMMAND_THRUST
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of file_logger_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void file_logger_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct NedCoor_f *acc = stateGetAccelNed_f();
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  //States: (no acc angular)
  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", acc->x, acc->y, acc->z);
  fprintf(file, "%f,%f,%f,%f,", quat->qi, quat->qx, quat->qy, quat->qz);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  //References: (quat no att) + acc angular
  fprintf(file, "%f,%f,%f,", GAStates.ref_pos_x, GAStates.ref_pos_y, GAStates.ref_pos_z);
  fprintf(file, "%f,%f,%f,", GAStates.ref_vel_x, GAStates.ref_vel_y, GAStates.ref_vel_z);
  fprintf(file, "%f,%f,%f,", GAStates.ref_acc_x, GAStates.ref_acc_y, GAStates.ref_acc_z);
  fprintf(file, "%f,%f,%f,%f,", GAStates.ref_quat_qi, GAStates.ref_quat_qx, GAStates.ref_quat_qy, GAStates.ref_quat_qz);
  fprintf(file, "%f,%f,%f,", GAStates.ref_rate_p, GAStates.ref_rate_q, GAStates.ref_rate_r);
  fprintf(file, "%f,%f,%f,", GAStates.ref_acc_p, GAStates.ref_acc_q, GAStates.ref_acc_r);
  //Filters: (quat no att, no pos, no vel) + acc angular
  fprintf(file, "%f,%f,%f,", GAStates.filt_acc_x, GAStates.filt_acc_y, GAStates.filt_acc_z);
  fprintf(file, "%f,%f,%f,%f,", GAStates.filt_quat_qi, GAStates.filt_quat_qx, GAStates.filt_quat_qy, GAStates.filt_quat_qz);
  fprintf(file, "%f,%f,%f,", GAStates.filt_rate_p, GAStates.filt_rate_q, GAStates.filt_rate_r);
  fprintf(file, "%f,%f,%f,", GAStates.filt_acc_p, GAStates.filt_acc_q, GAStates.filt_acc_r);
  fprintf(file, "%f,%f,%f,%f,", GAStates.motor_mixing_A1,GAStates.motor_mixing_A2,GAStates.motor_mixing_B1,GAStates.motor_mixing_B2);
  fprintf(file, "%f,%f,", GAStates.mode, GAStates.axes);

#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // Create output folder if necessary
  if (access(STRINGIFY(FILE_LOGGER_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(FILE_LOGGER_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[file_logger] Could not create log file directory %s.\n", STRINGIFY(FILE_LOGGER_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), date_time);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), date_time, counter);
    counter++;
  }

  file_logger = fopen(filename, "w");
  if(!file_logger) {
    printf("[file_logger] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[file_logger] Start logging to %s...\n", filename);

  file_logger_write_header(file_logger);
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file    */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  file_logger_write_row(file_logger);
}
