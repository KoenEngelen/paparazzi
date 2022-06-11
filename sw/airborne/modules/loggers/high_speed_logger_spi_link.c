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
  // count all periodic steps

  counter ++;

  // only send a new message if the previous was completely sent
  if (high_speed_logger_spi_link_ready) {
    high_speed_logger_spi_link_ready = false;

    high_speed_logger_spi_link_data.id = counter;




    if ((counter%2) == 1 ){
      float AoA = -aoa_adc.angle;
      float Voo = ms45xx.airspeed;
      float dp = ms45xx.pressure;
      struct NedCoor_f acc_body;
      struct FloatRates pqr = *stateGetBodyRates_f();
      acc_body.x = ACCEL_FLOAT_OF_BFP(imu.accel.x);
      acc_body.y = ACCEL_FLOAT_OF_BFP(imu.accel.y);
      acc_body.z = ACCEL_FLOAT_OF_BFP(imu.accel.z);

      high_speed_logger_spi_link_data.d1  = *(int32_t*)&AoA;
      high_speed_logger_spi_link_data.d2  = *(int32_t*)&Voo;
      high_speed_logger_spi_link_data.d3  = *(int32_t*)&dp;
      high_speed_logger_spi_link_data.d4  = *(int32_t*)&pqr.p;
      high_speed_logger_spi_link_data.d5  = *(int32_t*)&pqr.q;
      high_speed_logger_spi_link_data.d6  = *(int32_t*)&pqr.r;
      high_speed_logger_spi_link_data.d7  = *(int32_t*)&acc_body.x;
      high_speed_logger_spi_link_data.d8  = *(int32_t*)&acc_body.y;
      high_speed_logger_spi_link_data.d9  = *(int32_t*)&acc_body.z;
      high_speed_logger_spi_link_data.d10 = *(int32_t*)&sd_log[9];
      high_speed_logger_spi_link_data.d11 = *(int32_t*)&sd_log[10];
      high_speed_logger_spi_link_data.d12 = *(int32_t*)&sd_log[11];
      high_speed_logger_spi_link_data.d13 = aerobatic_active;
      high_speed_logger_spi_link_data.d14 = stabilization_cmd[COMMAND_ROLL];
      high_speed_logger_spi_link_data.d15 = stabilization_cmd[COMMAND_PITCH];
      aerobatic_active = 0;
    } else {
      
      struct NedCoor_f pos;
      struct NedCoor_f vel;
      struct NedCoor_f acc;
      struct FloatQuat q;

      pos = *stateGetPositionNed_f();
      vel = *stateGetSpeedNed_f();
      acc = *stateGetAccelNed_f();
      q   = *stateGetNedToBodyQuat_f();

      high_speed_logger_spi_link_data.d1  = *(int32_t*)&pos.x;
      high_speed_logger_spi_link_data.d2  = *(int32_t*)&pos.y;
      high_speed_logger_spi_link_data.d3  = *(int32_t*)&pos.z;
      high_speed_logger_spi_link_data.d4  = *(int32_t*)&vel.x;
      high_speed_logger_spi_link_data.d5  = *(int32_t*)&vel.y;
      high_speed_logger_spi_link_data.d6  = *(int32_t*)&vel.z;
      high_speed_logger_spi_link_data.d7  = *(int32_t*)&acc.x;
      high_speed_logger_spi_link_data.d8  = *(int32_t*)&acc.y;
      high_speed_logger_spi_link_data.d9  = *(int32_t*)&acc.z;
      high_speed_logger_spi_link_data.d10 = *(int32_t*)&q.qi;
      high_speed_logger_spi_link_data.d11 = *(int32_t*)&q.qx;
      high_speed_logger_spi_link_data.d12 = *(int32_t*)&q.qy;
      high_speed_logger_spi_link_data.d13 = *(int32_t*)&q.qz;
      high_speed_logger_spi_link_data.d14 = stabilization_cmd[COMMAND_YAW];
      high_speed_logger_spi_link_data.d15 = stabilization_cmd[COMMAND_THRUST];
    }
    
    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}