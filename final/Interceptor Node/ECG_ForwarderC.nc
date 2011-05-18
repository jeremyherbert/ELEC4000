#include <Timer.h>
#include "ECGHeader.h"
#include <IPDispatch.h>
#include <lib6lowpan.h>
#include <ip.h>
#include <string.h>

#include "printf.h"
#include "UDPReport.h"

#include "PrintfUART.h"

//#include "ApplicationDefinitions.h"
//#include "RssiDemoMessages.h"  


module ECG_ForwarderC {

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

	bool busy = FALSE;	
  	bool timerStarted;								
						
  	struct sockaddr_in6 route_dest;					


	event void Boot.booted() {
		call AMControl.start();
		call RadioControl.start();


		printfUART_init();							

		dbg("Boot", "booted: %i\n", TOS_NODE_ID);	
		call Status.bind(7001);						
	}



///////////////////// UDP ///////////////////////////////////
event void RadioControl.startDone(error_t e) {}

  event void RadioControl.stopDone(error_t e) {}

  event void Status.recvfrom(struct sockaddr_in6 *from, void *data, 
                             uint16_t len, struct ip_metadata *meta) {	
		route_dest = *from;
  }

 
  event void StatusTimer.fired() {}


void send_UPD_Beat_Message(BEAT_MSG* beatMsg){
         
        call Status.sendto(&route_dest, &beatMsg, sizeof(beatMsg));
 	printf("Beat Recieved From : %i\n", beatMsg->NODE_ID);
        printfflush();


}

void send_UPD_ECG_Message(ECG_PACKET* ECGMsg){
         
        call Status.sendto(&route_dest, &ECGMsg, sizeof(ECGMsg));
 	printf("%i %i %i %i %i %i %i %d \n", 		
		ECGMsg->D1 , 
 		ECGMsg->D2 ,
 		ECGMsg->D3 ,
 		ECGMsg->D4 ,
 		ECGMsg->D5 ,
 		ECGMsg->D6 ,
 		ECGMsg->D7 ,
 		ECGMsg->TIME);
	 printfflush();


}



////////////////////////////////////////////////////////////





///////////////////// Radio /////////////////////////


  

	/* Recieves and AM Message, checks for validity, strips out the rssi, 
		and calls the send udp function
	*/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    	
	BEAT_MSG *beatMsg; 
        ECG_PACKET *ECGMsg;

		
		if(len == sizeof(BEAT_MSG)){	    
			beatMsg = (BEAT_MSG*) payload;
			send_UPD_Beat_Message(beatMsg);

		}
		
		if(len == sizeof(ECG_PACKET)){
			ECGMsg = (ECG_PACKET*) payload;
			printf("ECG DATA RECEIVED FROM : %i \n", beatMsg->NODE_ID);
        		printfflush();			
			
			send_UPD_ECG_Message(ECGMsg);
			

		}

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


////////////////////////////////////////////////////////////////////


}
