/*
 * Copyright (C) 2005-2013 The Paparazzi Team
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

#include "high_speed_logger_spi_link.h"
#include "guidance/guidance_aerobatic.h"
#include "stabilization.h"
#include "subsystems/imu.h"
#include "modules/sensors/airspeed_ms45xx_i2c.h"
#include "modules/sensors/aoa_adc.h"
#include "mcu_periph/spi.h"
#include "state.h"


struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool high_speed_logger_spi_link_ready = true;

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans);

void high_speed_logger_spi_link_init(void)
{
  high_speed_logger_spi_link_data.id = 0;

  high_speed_logger_spi_link_transaction.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction.output_length = sizeof(high_speed_logger_spi_link_data);
  high_speed_logger_spi_link_transaction.output_buf    = (uint8_t *) &high_speed_logger_spi_link_data;
  high_speed_logger_spi_link_transaction.input_length  = 0;
  high_speed_logger_spi_link_transaction.input_buf     = NULL;
  high_speed_logger_spi_link_transaction.after_cb      = high_speed_logger_spi_link_trans_cb;
}


void high_speed_logger_spi_link_periodic(void)
{
  // Static counter to identify missing samples
  static int32_t counter = 0;
  static volatile bool write_extension = false;
  // count all periodic steps
  counter ++;

  // only send a new message if the previous was completely sent
  if (high_speed_logger_spi_link_ready && !write_extension) {
    // copy the counter into the SPI datablock
    high_speed_logger_spi_link_data.id = counter;

    high_speed_logger_spi_link_ready = false;
    write_extension = true;

    //return correct
    high_speed_logger_spi_link_data.qi = (int32_t)1000 * aerolog.qi;
    high_speed_logger_spi_link_data.qx = (int32_t)1000 * aerolog.qx;
    high_speed_logger_spi_link_data.qy = (int32_t)1000 * aerolog.qy;
    high_speed_logger_spi_link_data.qz = (int32_t)1000 * aerolog.qz;

    //return 0
    high_speed_logger_spi_link_data.ax = (int32_t)1000 * aerolog.pos_x;
    high_speed_logger_spi_link_data.ay = (int32_t)1000 * aerolog.pos_y;
    high_speed_logger_spi_link_data.az = (int32_t)1000 * aerolog.pos_z;
    
    #ifdef SITL
    printf("\n\nLOGPOS: %e %e %e\n", high_speed_logger_spi_link_data.ax,high_speed_logger_spi_link_data.ay,high_speed_logger_spi_link_data.az);
    printf("\npos: %e %e %e",(int32_t)1000* aerolog.pos_x, (int32_t)1000*aerolog.pos_y, (int32_t)1000*aerolog.pos_z);
    #endif

    //return 0
    high_speed_logger_spi_link_data.bx = (int32_t)1000 * aerolog.vel_x;
    high_speed_logger_spi_link_data.by = (int32_t)1000 * aerolog.vel_y;
    high_speed_logger_spi_link_data.bz = (int32_t)1000 * aerolog.vel_z;

    //return 0
    high_speed_logger_spi_link_data.cx = (int32_t)1000 * aerolog.acc_x;
    high_speed_logger_spi_link_data.cy = (int32_t)1000 * aerolog.acc_y;
    high_speed_logger_spi_link_data.cz = (int32_t)1000 * aerolog.acc_z;
    
    //return correct
    high_speed_logger_spi_link_data.dx = (int32_t)1000 * aerolog.t;
    high_speed_logger_spi_link_data.dy = (int32_t)1000 * aerolog.mode;

  }
  else if(high_speed_logger_spi_link_ready && write_extension){
    // copy the counter into the SPI datablock
    high_speed_logger_spi_link_data.id = counter+1000;

    high_speed_logger_spi_link_ready = false;
    write_extension = false;

    //return 0
    high_speed_logger_spi_link_data.qi = stabilization_cmd[COMMAND_ROLL];
    high_speed_logger_spi_link_data.qx = stabilization_cmd[COMMAND_PITCH];
    high_speed_logger_spi_link_data.qy = stabilization_cmd[COMMAND_YAW];
    high_speed_logger_spi_link_data.qz = stabilization_cmd[COMMAND_THRUST];
    
    //return correct
    high_speed_logger_spi_link_data.ax = (int32_t)1000 * aerolog.ref_pos_x;
    high_speed_logger_spi_link_data.ay = (int32_t)1000 * aerolog.ref_pos_y;
    high_speed_logger_spi_link_data.az = (int32_t)1000 * aerolog.ref_pos_z;
    
    //return 0 (which it should be)
    high_speed_logger_spi_link_data.bx = (int32_t)1000 * aerolog.ref_vel_x;
    high_speed_logger_spi_link_data.by = (int32_t)1000 * aerolog.ref_vel_y;
    high_speed_logger_spi_link_data.bz = (int32_t)1000 * aerolog.ref_vel_z;
    
    //return 0 (which it should be)
    high_speed_logger_spi_link_data.cx = (int32_t)1000 * aerolog.ref_acc_x;
    high_speed_logger_spi_link_data.cy = (int32_t)1000 * aerolog.ref_acc_y;
    high_speed_logger_spi_link_data.cz = (int32_t)1000 * aerolog.ref_acc_z;

    //return correct
    high_speed_logger_spi_link_data.dx = (int32_t)1000 * aerolog.max_vel;
    high_speed_logger_spi_link_data.dy = (int32_t)1000 * aerolog.max_acc;

    
  }

  // if (autopilot.in_flight){
    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  // }
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}


