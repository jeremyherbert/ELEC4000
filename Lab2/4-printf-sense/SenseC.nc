/*
 * Copyright (c) 2006, Technische Universitaet Berlin
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
 * $Revision: 1.4 $
 * $Date: 2006/12/12 18:22:49 $
 * @author: Jan Hauer
 * ========================================================================
 */

/**
 * 
 * Sensing demo application. See README.txt file in this directory for usage
 * instructions and have a look at tinyos-2.x/doc/html/tutorial/lesson5.html
 * for a general tutorial on sensing in TinyOS.
 *
 * @author Jan Hauer
 */

#include "Timer.h"
#include "printf.h"

module SenseC
{
  uses {
    interface Boot;
    interface Leds;
    interface Timer<TMilli>;
    interface Timer<TMilli> as TimerPrint;
    interface Read<uint16_t> as ReadTsrC;
    interface Read<uint16_t> as ReadParC;
    interface Read<uint16_t> as ReadSht11CTemp;
    interface Read<uint16_t> as ReadSht11CHum;
    interface Read<uint16_t> as ReadDemo;
  }
}
implementation
{
  // sampling frequency in binary milliseconds
  #define SAMPLING_FREQUENCY 100
  
  uint16_t parC = 0;
  uint16_t tsrC = 0;
  uint16_t sht_temp = 0;
  uint16_t sht_hum = 0;
  uint16_t demo = 0;

  event void Boot.booted() {
    call Timer.startPeriodic(SAMPLING_FREQUENCY);
    call TimerPrint.startPeriodic(500); // run every 500 ms
    printf("started");
    printfflush();
  }

  event void Timer.fired() 
  {
    call ReadTsrC.read();
    call ReadParC.read();
    call ReadSht11CTemp.read();
    call ReadSht11CHum.read();
    call ReadDemo.read();
  }

  event void TimerPrint.fired()
  {
    printf("tsr: %i; par: %i; sht_temp: %i, sht_hum: %i; demo: %i\n", tsrC, parC, sht_temp, sht_hum, demo);
    printfflush();
  }

  // sensor reads

  event void ReadParC.readDone(error_t result, uint16_t data) 
  {
    if (result == SUCCESS) parC = data;
  }

  event void ReadTsrC.readDone(error_t result, uint16_t data)
  {
    if (result == SUCCESS) tsrC = data;
  }

  event void ReadSht11CTemp.readDone(error_t result, uint16_t data)
  {
    if (result == SUCCESS) sht_temp = data;
  }

  event void ReadSht11CHum.readDone(error_t result, uint16_t data)
  { 
    if (result == SUCCESS) sht_hum = data;
  }

  event void ReadDemo.readDone(error_t result, uint16_t data)
  {
    if (result == SUCCESS) demo = data;
  }
}
