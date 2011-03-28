#include <Timer.h>
#include "RssiMessage.h"
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


  	bool timerStarted;								//UDP
  	nx_struct udp_report stats;						//UDP
  	struct sockaddr_in6 route_dest;					//UDP
	


	event void Boot.booted() {
		call AMControl.start();
		call RadioControl.start();


		printfUART_init();							//UDP

		dbg("Boot", "booted: %i\n", TOS_NODE_ID);	//UDP
		call Status.bind(7001);						//UDP
	}



///////////////////// UDP ///////////////////////////////////
event void RadioControl.startDone(error_t e) {}

  event void RadioControl.stopDone(error_t e) {}

  event void Status.recvfrom(struct sockaddr_in6 *from, void *data, 
                             uint16_t len, struct ip_metadata *meta) {	
		route_dest = *from;
  }

 
  event void StatusTimer.fired() {}

/* 
	Takes a mesage and an rssi, extracts the data and sends
	it through to the UDP connection.

	Note, if you jsut want to access the data for manipulation
	it can be found at
	
	rssiMsg->NODE_ID
	rssiMsg->seq_num
	rssibase;

*/
void send_UPD_Message(RssiMsg* rssiMsg, uint16_t rssibase){
	char header[18] = "---------------\r\n";	
	char n_mess[10] = "Node ID :";
	char nodeid[3];	
	char seq_mess[10] = "Seq_Num :";
	char seqnum[4];
	char ss_mess[18] = "Signal Strength :";
	char rssi [3];
	char test1 [14] ;
	char test2[14];
	char test3 [22] ;

	char newline[3] = "\r\n";
	

	// Convert Message numbers to ascii
	itoa(rssiMsg->NODE_ID, nodeid, 10);
	itoa(rssiMsg->seq_num, seqnum, 10);
	itoa(rssibase ,rssi,  10);

	// Concatinate corrosponding data to each message
	strcpy(test1, n_mess);
	strcat(test1, nodeid);
	strcat(test1, newline);

	strcpy(test2, seq_mess);
	strcat(test2, seqnum);
	strcat(test2, newline);


	strcpy(test3, ss_mess);
	strcat(test3, rssi);
	strcat(test3, newline);


	// Send each message
	call Status.sendto(&route_dest, header, sizeof(header));
	call Status.sendto(&route_dest, test1, sizeof(test1));
	call Status.sendto(&route_dest, test2, sizeof(test2));
	call Status.sendto(&route_dest, test3, sizeof(test3));


}


////////////////////////////////////////////////////////////






	///////////////////// Radio /////////////////////////

	uint16_t getRssi(message_t *msg);
  

	/* Recieves and AM Message, checks for validity, strips out the rssi, 
		and calls the send udp function
	*/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    	RssiMsg *rssiMsg = (RssiMsg*) payload;
    	uint16_t rssibase = getRssi(msg);
		
		if(len == sizeof(RssiMsg)){	    

			if (rssiMsg->NODE_ID == 33){		
				call Leds.led0Toggle();
			} else {
				call Leds.led1Toggle();
			}
			
			send_UPD_Message(rssiMsg, rssibase);

		} else {
			call Leds.led2Toggle();
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

	uint16_t getRssi(message_t *msg){
		return (uint16_t) call CC2420Packet.getRssi(msg);
	}




////////////////////////////////////////////////////////////////////


}
