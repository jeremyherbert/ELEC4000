/*
 * "Copyright (c) 2008 The Regents of the University  of California.
 * All rights reserved."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

#include <6lowpan.h>
#include <Timer.h>
#include "Fireworks.h"

configuration TCPEchoC {

} implementation {
  components MainC, LedsC;
  components TCPEchoP;
	
  TCPEchoP.Boot -> MainC;
  TCPEchoP.Leds -> LedsC;

///////// Fireworks

	components FireworksC as App;
	components TimeSyncC;
	
	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;
	App.Boot -> MainC;

	components new TimerMilliC() as ledTimer;
  App.LedTimer -> ledTimer;

  App.GlobalTime -> TimeSyncC;
  App.TimeSyncInfo -> TimeSyncC;
  App.Leds -> LedsC;

	MainC.SoftwareInit -> TimeSyncC;
	TimeSyncC.Boot -> MainC;

///////// Active message

	components ActiveMessageC;

  App.RadioControl -> ActiveMessageC;
  App.Receive -> ActiveMessageC.Receive[AM_FIREWORKS_MSG];
  App.AMSend -> ActiveMessageC.AMSend[AM_FIREWORKS_MSG];
  App.Packet -> ActiveMessageC;
  App.AMPacket -> ActiveMessageC;
  App.PacketTimeStamp -> ActiveMessageC;

//////// UDP

  components new TimerMilliC();
  components IPDispatchC;

  TCPEchoP.RadioControl -> IPDispatchC;
  components new UdpSocketC() as Echo,
    new UdpSocketC() as Status;
  TCPEchoP.Echo -> Echo;

  components new TcpSocketC() as TcpEcho;
  TCPEchoP.TcpEcho -> TcpEcho;

  components new TcpSocketC() as TcpWeb, HttpdP;
  components new SensirionSht11C() as Sht11C;
  components new TimerMilliC() as TimerTemp;

  HttpdP.Boot -> MainC;
  HttpdP.Leds -> LedsC;
  HttpdP.Tcp -> TcpWeb;
  HttpdP.TimerTemp -> TimerTemp;
  HttpdP.ReadTemp -> Sht11C.Temperature;

  TCPEchoP.Status -> Status;

  TCPEchoP.StatusTimer -> TimerMilliC;

  components UdpC;

  TCPEchoP.IPStats -> IPDispatchC.IPStats;
  TCPEchoP.RouteStats -> IPDispatchC.RouteStats;
  TCPEchoP.ICMPStats -> IPDispatchC.ICMPStats;
  TCPEchoP.UDPStats -> UdpC;

  components RandomC;
  TCPEchoP.Random -> RandomC;

  components UDPShellC;
}
