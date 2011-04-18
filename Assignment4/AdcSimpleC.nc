/* 
 * Copyright (c) 2007, Technische Universitaet Berlin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the Technische Universitaet Berlin nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * - Revision -------------------------------------------------------------
 * $Revision: 1.1 $
 * $Date: 2007/08/01 09:28:58 $
 * @author: Jan Hauer <hauer@tkn.tu-berlin.de>
 * ========================================================================
 */

#include "Msp430Adc12.h"
#include "printf.h"

module AdcSimpleC {
  provides {
    interface AdcConfigure<const msp430adc12_channel_config_t*> as VoltageConfigure;
  }
  uses {
    interface Boot;
    interface Read<uint16_t> as VoltageRead;
    interface Leds;
    interface Timer<TMilli>;
  }
}
implementation {

  const msp430adc12_channel_config_t config = {
      /*inch: SUPPLY_VOLTAGE_HALF_CHANNEL,*/
      inch: INPUT_CHANNEL_A3,
      sref: REFERENCE_VREFplus_AVss,
      ref2_5v: REFVOLT_LEVEL_1_5,
      adc12ssel: SHT_SOURCE_ACLK,
      adc12div: SHT_CLOCK_DIV_1,
      sht: SAMPLE_HOLD_4_CYCLES,
      sampcon_ssel: SAMPCON_SOURCE_SMCLK,
      sampcon_id: SAMPCON_CLOCK_DIV_1
  };

  // sampling frequency in binary milliseconds
  #define SAMPLING_FREQUENCY 15

  #define THRESHOLD 300
  uint8_t sampling = 0;
  uint8_t level = 0;

  event void Boot.booted() {
	call Leds.led0Off();
	call Leds.led1Off();
	call Leds.led2Off();
	call Timer.startPeriodic(SAMPLING_FREQUENCY);
  }


  event void VoltageRead.readDone( error_t result, uint16_t val )
  {
    if (result == SUCCESS){
        if (sampling) {
            printf("%i\n", val);
            call Leds.led0On();
            if (val > THRESHOLD) {
                if (level < 255) level++;
            } else {
                if (level > 0) level--;
            }
        call VoltageRead.read();
        }


//	printf("Here is a uint16: %u\n", val);
//	printfflush();
    }
   }

  async command const msp430adc12_channel_config_t* VoltageConfigure.getConfiguration()
  {
    return &config; // must not be changed
  }

  event void Timer.fired() 
  {
        sampling ^= 0x01;

        if (sampling == 0) {
            printf("l:%i\n", level);
            if ((level >= 2) && (level < 15)) {
                printf("clap\n");
                printfflush();
                level = 0;
            }
        }

        call VoltageRead.read();
  }

}
