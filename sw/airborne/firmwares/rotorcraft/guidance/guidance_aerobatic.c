/*
 * Copyright (C) 2015
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
 */

/** @file firmwares/rotorcraft/guidance/guidance_aerobatic.c
 *  Reference generation for horizontal guidance.
 *
 */


float guidance_aerobatic_indi_gain = 0.5;
float debug_value;


#include "subsystems/datalink/telemetry.h"

 #define PPRZ_MSG_ID_AEROBATIC_INDI 158

 static void send_aerobatic_indi(struct transport_tx *trans, struct link_device *dev)
 {
   pprz_msg_send_AEROBATIC_INDI(trans, dev, AC_ID,
                         &debug_value, 
                         &guidance_aerobatic_indi_gain);
 }


 void guidance_aerobatic_init(void) {

   debug_value = 0.0;


  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AEROBATIC_INDI, send_aerobatic_indi);]
 }


 void guidance_aerobatic_run(void) {
   debug_value += 0.00195;

   if (debug_value > 60){
     debug_value = 0;
   }

 }
