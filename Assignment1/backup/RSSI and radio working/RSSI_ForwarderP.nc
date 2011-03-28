#include <Timer.h>
#include "RSSI_Forwarder.h"



#include <IPDispatch.h>
#include <lib6lowpan.h>
#include <ip.h>

#include "UDPReport.h"
#include "PrintfUART.h"

//#include "ApplicationDefinitions.h"
//#include "RssiDemoMessages.h"  


module RSSI_ForwarderP {

	uses{
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as Timer0;
		
		//////////RADIO//////////////		
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;
		interface Receive;
		


		///////////UDP//////////////
		interface SplitControl as RadioControl;
		interface UDP as Status;
		interface Timer<TMilli> as StatusTimer;


	}
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


  	bool timerStarted;							//UDP
  	nx_struct udp_report stats;					//UDP
  	struct sockaddr_in6 route_dest;				//UDP
	


	event void Boot.booted() {
		call AMControl.start();
		call RadioControl.start();


    printfUART_init();							//UDP

    dbg("Boot", "booted: %i\n", TOS_NODE_ID);	//UDP
    call Status.bind(7001);						//UDP
	}



///////////////////// UDP ///////////////////////////////////
event void RadioControl.startDone(error_t e) {

  }

  event void RadioControl.stopDone(error_t e) {

  }

  event void Status.recvfrom(struct sockaddr_in6 *from, void *data, 
                             uint16_t len, struct ip_metadata *meta) {
		
		route_dest = *from;
  
  }

 
  event void StatusTimer.fired() {

    if (!timerStarted) {
      timerStarted = TRUE;
    }

    stats.seqno++;
    stats.sender = TOS_NODE_ID;
 
    stats.temp = 1;



    call Status.sendto(&route_dest, &stats, sizeof(stats));
  }


void send_UPD_Message(RssiMsg* rssiMsg){
	char tempe[19] = "Signal Strength : \0";
	char temp [4];
	/*
	char temp_data[10];
	char* temps;
	uint16_t count;
	uint16_t temp;  
	uint16_t dummy;
	uint16_t i;
	
	count = 0;	
	temp = (rssiMsg->rssi);
	for(i=0; i < 10; i++){
		dummy = temp % 10;
		temp_data[count++] = (char)(dummy+48);
		temp = temp /  10;
		if (temp <= 0) break;
	}

	for(i=0; i < count; i++){
		temps[i] = temp_data[count - i];
	}*/
	itoa(rssiMsg->rssi ,temp,  10);

	temp[2] = '\r';
	temp[2] = '\n';
	
	call Status.sendto(&route_dest, tempe, sizeof(tempe));
    call Status.sendto(&route_dest, &temp, sizeof(temp));

}


////////////////////////////////////////////////////////////






	///////////////////// Radio /////////////////////////

	uint16_t getRssi(message_t *msg);
  


	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    	RssiMsg *rssiMsg = (RssiMsg*) payload;
    	rssiMsg->rssi = getRssi(msg);
	
		call Leds.led0Toggle();
    	
		send_UPD_Message(rssiMsg);

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



////////////////////////////////////////////////////////////////////


}
