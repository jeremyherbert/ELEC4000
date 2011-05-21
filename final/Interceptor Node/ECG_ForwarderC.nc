#include <Timer.h>
#include "ECGHeader.h"
#include <IPDispatch.h>
#include <lib6lowpan.h>
#include <ip.h>
#include <string.h>

#include "printf.h"
#include "UDPReport.h"

#include "PrintfUART.h"

module ECG_ForwarderC {

	uses{
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TimeKeeper;
		interface GlobalTime<TMilli>;		


		//////////RADIO//////////////		
		interface Packet;
		interface AMPacket;
		interface AMSend  as ECGMsgSend;
		interface SplitControl as AMControl;
		interface Receive;


		///////////UDP//////////////
		interface SplitControl as RadioControl;
		interface UDP as Status;


	}





} implementation {

  	  nx_struct udp_report stats;
 	 struct sockaddr_in6 route_dest;

	bool busy = FALSE;	
  	bool timerStarted;								
						

  			


	event void Boot.booted() {
		call AMControl.start();
		call RadioControl.start();


		printfUART_init();

    		route_dest.sin6_port = hton16(7000);
    		inet_pton6(REPORT_DEST, &route_dest.sin6_addr);							

		dbg("Boot", "booted: %i\n", TOS_NODE_ID);	
		call Status.bind(7001);						
	}


void updateNodes(void *data, uint16_t len){

    call ECGMsgSend.send(AM_BROADCAST_ADDR, &data, len);
}


///////////////////// UDP ///////////////////////////////////
event void RadioControl.startDone(error_t e) {}

  event void RadioControl.stopDone(error_t e) {}

  event void Status.recvfrom(struct sockaddr_in6 *from, void *data, 
                             uint16_t len, struct ip_metadata *meta) {	
		route_dest = *from;
  
	if(len == 0){
		updateNodes(data, len);
	}



}

 
  event void TimeKeeper.fired() {
	uint32_t currGlobTime = call GlobalTime.getLocalTime();
        call GlobalTime.getGlobalTime(&currGlobTime);

	call Status.sendto(&route_dest, &currGlobTime, sizeof(currGlobTime));

  }


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

	event void ECGMsgSend.sendDone(message_t* msg, error_t error){
	}

	





////////////////////////////////////////////////////////////////////


}
