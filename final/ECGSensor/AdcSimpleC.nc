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
    interface Timer<TMilli> as RTimer;
    interface LocalTime<TMilli>;
    interface Timer<TMilli> as MovementDelayTimer;
    interface Timer<TMilli> as AlarmTimer;
    interface Timer<TMilli> as SampleTimer;
  }
}
implementation {

  const msp430adc12_channel_config_t config = {
      /*inch: SUPPLY_VOLTAGE_HALF_CHANNEL,*/
      inch: INPUT_CHANNEL_A0,
      sref: REFERENCE_VREFplus_AVss,
      ref2_5v: REFVOLT_LEVEL_1_5,
      adc12ssel: SHT_SOURCE_ACLK,
      adc12div: SHT_CLOCK_DIV_1,
      sht: SAMPLE_HOLD_4_CYCLES,
      sampcon_ssel: SAMPCON_SOURCE_SMCLK,
      sampcon_id: SAMPCON_CLOCK_DIV_1
  };
  
  #define MOVEMENT_THRESHOLD        200
  #define ADC_BUFFER_SIZE           1000

  void dumpAdc();

  uint8_t init = 0;
  uint8_t r_timer_ok = 0; // windowed r wave detection
  uint8_t r_period = 0; // adaptive r wave detection
  uint16_t r_threshold = 2500; // ADC threshold for r wave detection
  uint8_t r_flag = 0; // are we currently in an r wave ?
  
  uint8_t movement_error = 0; // is the patient moving ?
  uint16_t movement_delay = 5000; // how long should we wait before trying again ?

  uint16_t alarm_interval = 5000; // number of ms before alarm goes off

  uint8_t sample_interval = 5; // sample every n ms
  
  uint16_t adc_buffer[ADC_BUFFER_SIZE]; 
  uint16_t adc_buffer_index = 0;

  event void Boot.booted() {
	call Leds.led0Off();
	call Leds.led1Off();
	call Leds.led2Off();
	call RTimer.startOneShot(MOVEMENT_THRESHOLD);
        call SampleTimer.startPeriodic(sample_interval);
  }

  void restartAlarmTimer()
  {
    call AlarmTimer.stop();
    call AlarmTimer.startOneShot(alarm_interval);
  }

  event void VoltageRead.readDone( error_t result, uint16_t val )
  {
    if (result == SUCCESS){
        if (movement_error == 0) {
            if ((val > r_threshold) && (r_flag == 0)) {
                if ((r_timer_ok == 1) || (init == 0)) {
                    // r wave detected
                    r_flag = 1;
                    restartAlarmTimer();
                    init = 1; // intialised
                    call Leds.led0On();
                    call Leds.led1Off(); // no error
                } else {
                    call Leds.led1On(); // movement error
                    movement_error = 1;
                    call AlarmTimer.stop();
                    call MovementDelayTimer.startOneShot(movement_delay);
                }
                r_timer_ok = 0;
                if (movement_error == 0) call RTimer.startOneShot(MOVEMENT_THRESHOLD);
            
            } else {
                r_flag = 0;
                call Leds.led0Off();
            }
        
        }
        if (adc_buffer_index < 1000) {
            adc_buffer[adc_buffer_index++] = val;
        } else {
            dumpAdc();
        }
        printf(" ");
        printfflush();
    }

   }

  async command const msp430adc12_channel_config_t* VoltageConfigure.getConfiguration()
  {
    return &config; // must not be changed
  }

  void dumpAdc()
  {
    call SampleTimer.stop();
    call AlarmTimer.stop();
    
    uint16_t i=0;
    for (i=0; i<1000; i++) {
        printf("%u,", adc_buffer[i]);
    }
    printfflush();

    adc_buffer_index = 0;
    call SampleTimer.startPeriodic(sample_interval);
    restartAlarmTimer();
  }

  event void SampleTimer.fired()
  {
    call VoltageRead.read();
  }

  event void RTimer.fired() 
  {
    r_timer_ok = 1;
  }

  event void MovementDelayTimer.fired()
  {
    movement_error = 0;
    restartAlarmTimer();
  }

  event void AlarmTimer.fired()
  {
    call Leds.led2On();
  }
}
