//#include <6lowpan.h>
//#include "message.h"
#include <Timer.h>
#include "RSSI_Forwarder.h"

module RSSI_ForwarderP {


	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli> as Timer0;
	uses interface Packet;
	uses interface AMPacket;
	uses interface AMSend;
	uses interface SplitControl as AMControl;
	uses interface Receive;

#ifdef __CC2420_H__
  uses interface CC2420Packet;
#elif defined(TDA5250_MESSAGE_H)
  uses interface Tda5250Packet;    
#else
  uses interface PacketField<uint8_t> as PacketRSSI;
#endif 



} implementation {

	uint16_t counter = 0;
	bool busy = FALSE;	
	//message_t pkt;

	event void Boot.booted() {
		call AMControl.start();
	}

	uint16_t getRssi(message_t *msg);
  


	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    	RssiMsg *rssiMsg = (RssiMsg*) payload;
    	rssiMsg->rssi = getRssi(msg);
	
		call Leds.led0Toggle();
    
		return msg;	
	}


	event void AMControl.startDone(error_t err){
		if (err == SUCCESS) {
		} else {
			call AMControl.start();
		}
	}

	event void AMControl.stopDone(error_t err) {
	}

	event void AMSend.sendDone(message_t* msg, error_t error){
	}

	event void Timer0.fired(){

	}


#ifdef __CC2420_H__  
  uint16_t getRssi(message_t *msg){
    return (uint16_t) call CC2420Packet.getRssi(msg);
  }
#elif defined(CC1K_RADIO_MSG_H)
    uint16_t getRssi(message_t *msg){
    cc1000_metadata_t *md =(cc1000_metadata_t*) msg->metadata;
    return md->strength_or_preamble;
  }
#elif defined(PLATFORM_IRIS)
  uint16_t getRssi(message_t *msg){
    if(call PacketRSSI.isSet(msg))
      return (uint16_t) call PacketRSSI.get(msg);
    else
      return 0xFFFF;
  }
#elif defined(TDA5250_MESSAGE_H)
   uint16_t getRssi(message_t *msg){
       return call Tda5250Packet.getSnr(msg);
   }
#else
  #error Radio chip not supported! This demo currently works only \
         for motes with CC1000, CC2420, RF230 or TDA5250 radios.  
#endif


}
