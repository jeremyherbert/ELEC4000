#include "../ECGHeader.h"
#include <Timer.h>
#include <UserButton.h>
#include "printf.h"

module patientC {
  	uses{
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as ReadTimer;
		interface Timer<TMilli> as HeartBeatTimer;
		interface GlobalTime<TMilli>;
		  
		interface Packet;
		interface AMPacket;
		interface Receive;
		interface AMSend as ECGMsgSend;
		interface SplitControl as RadioControl;
	
	

		interface LogRead;
	   	interface LogWrite;

		interface Notify<button_state_t>;


  	}
} implementation {
  	message_t msg;
  
  	uint16_t ID;
  	uint16_t BEAT_INTERVAL_MS = 15000;
  	uint16_t READ_INTERVAL_MS = 1500;

  	uint16_t seq = 0;
  	bool m_busy = TRUE;
  	ECG_DATA m_entry;

  	bool LIVE = FALSE;


  	event void Boot.booted(){
    	call RadioControl.start();
    	ID = TOS_NODE_ID;
    	call Notify.enable();
    	printf("started\n");
    	printfflush();

  	}


	
	event void LogRead.seekDone(error_t err) {
    if (call LogRead.read(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
		


    };

	}

  //Send message (this will currently loop, sending one message for each peice of data stored
  // Innefecient, need to ask about sending large messages
  	void sendMessage(void * buf){
	
		ECG_DATA *ECGData = (ECG_DATA*) buf;
		ECG_PACKET *m_packet = (ECG_PACKET*)(call Packet.getPayload(&msg, sizeof (ECG_PACKET)));
		
		m_packet->NODE_ID = ID;
		m_packet->D1 = ECGData->D1;
 		m_packet->D2 = ECGData->D2;
 		m_packet->D3 = ECGData->D3;
 		m_packet->D4 = ECGData->D4;
 		m_packet->D5 = ECGData->D5;
 		m_packet->D6 = ECGData->D6;
 		m_packet->D7 = ECGData->D7;
 		m_packet->TIME = ECGData->TIME;
	
		printf("SENDING ECG PACKET\n");
		printfflush();

		call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(ECG_PACKET)); 

  	}
  
  
 	void sendEmergency(uint8_t type){
		uint32_t currGlobTime = call GlobalTime.getLocalTime();
 
		EMER_MSG *m_packet = (EMER_MSG*)(call Packet.getPayload(&msg, sizeof (EMER_MSG)));
	
		call GlobalTime.getGlobalTime(&currGlobTime);
		
		m_packet->NODE_ID = ID;
		m_packet->TYPE = type;
  		m_packet->TIME = currGlobTime;
	
		printf("SENDING Emergency PACKET\n");
		printfflush();

		call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(EMER_MSG)); 

  }
  
  
   //Currently, on button press, read flash and send to base
 //Change this to be either timed or when flash getting full
	event void Notify.notify (button_state_t state) {
		sendEmergency(1);
		call LogWrite.erase();
	}

  

  //Event fired after each read from the flash, (ie, each full struct of ECG data is read 
 	event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
    	printf("read done!\n");
    	if ( (len == sizeof(ECG_DATA)) && (buf == &m_entry) ) {
		   //call Send.send(&m_entry.msg, m_entry.len);

	
		  /* CHANGE THIS TO SEND MESSAGE VIA RADIIO */
		  
		  	sendMessage(buf);
		  
		  	call Leds.led1On();
		  	call LogRead.read(&m_entry, sizeof(ECG_DATA));
    	
    	} else {
		  	if (call LogWrite.erase() != SUCCESS) {
			// Handle error.
		  	}
          
      
      
      		call Leds.led1Off();
      		call Leds.led0On();
    	}
  	}




	event void LogWrite.eraseDone(error_t err) {
    	uint32_t currGlobTime = call GlobalTime.getLocalTime();
        
    	REQUEST_DONE_MSG* recDoneMsg = (REQUEST_DONE_MSG*)(call Packet.getPayload(&msg, sizeof (REQUEST_DONE_MSG)));
    
    	call GlobalTime.getGlobalTime(&currGlobTime);
    
    	if (err == SUCCESS) {
      		m_busy = FALSE;
    	} else {
      // Handle error.
    	}
    
    	call Leds.led0Off();
    
    
  	
  		printf("Request Done \n");
    	printfflush();

  		recDoneMsg->NODE_ID = ID;
    	recDoneMsg->TIME = currGlobTime;


    	call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(REQUEST_DONE_MSG));
    
    
  	}


  	event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
    	m_busy = FALSE;
    	call Leds.led2Off();
  	}



  	event void LogWrite.syncDone(error_t err) {
  	}





  	event void RadioControl.startDone(error_t result){
    
    	call HeartBeatTimer.startOneShot(BEAT_INTERVAL_MS);
 	}

  	event void RadioControl.stopDone(error_t result){}

  	event void HeartBeatTimer.fired(){
    	BEAT_MSG * beat_msg = (BEAT_MSG*)(call Packet.getPayload(&msg, sizeof (BEAT_MSG)));
  	
  		printf("BEAT \n");
    	printfflush();
  	
    	beat_msg->NODE_ID = ID;

    	call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(BEAT_MSG));
    	call HeartBeatTimer.startOneShot(BEAT_INTERVAL_MS);
  	}


  	void readRawECGData(){
  
  		uint32_t currGlobTime = call GlobalTime.getLocalTime();
    	call GlobalTime.getGlobalTime(&currGlobTime);
    

		m_entry.D1 =100;
	 	m_entry.D2 =19;
	 	m_entry.D3 =167;
	 	m_entry.D4 =188;
	 	m_entry.D5 =16;
	 	m_entry.D6 =166;
	 	m_entry.D7 =1076;
	 	m_entry.TIME =currGlobTime;



  	}
  

  //Dummy data, should read from ecg. Reads ECG data, stores on flash.
  //Need to implement local checking to test for emrgency.
  	event void ReadTimer.fired(){
	

	
		if(LIVE == TRUE){
			if (!m_busy) {

	 			readRawECGData();
		
				m_busy = TRUE;
				if (call LogWrite.append(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
			   	 m_busy = FALSE;
			    	printf("write error");
			    	printfflush();
				}
			}
			call ReadTimer.startOneShot(READ_INTERVAL_MS);
		}
	
  	}

	// recieves a message 
    /*Test for message from basestation. This causes the patient node to set its flag to live
      This enables it to start storing data. Origanlly live will be set to false, however teh beat will
      start from the moment the node has initalised. This is to allow the basestation to identify that 
      the node is ready to start, but will not cause problems before the node and patient information has been specified
      at the basestation.*/
    event message_t* Receive.receive(message_t* msgPtr, void* payload, uint8_t len)
    {
		UPDATE_PATIENT_PACKET * m_packet;

		printf("Message recieved  ", TOS_NODE_ID);
				printfflush();

		if(len == sizeof(UPDATE_PATIENT_PACKET)) {
			m_packet = (UPDATE_PATIENT_PACKET*)payload;

			printf("Data %i  %i  %i  %i \n",m_packet->NODE_ID , m_packet->READ_INTERVAL, m_packet->BEAT_INTERVAL,
								 m_packet->IS_LIVE );
			printfflush();

				if(m_packet ->NODE_ID == TOS_NODE_ID){
					if(m_packet->READ_INTERVAL != 0) READ_INTERVAL_MS = m_packet->READ_INTERVAL;
				    if(m_packet->BEAT_INTERVAL != 0) BEAT_INTERVAL_MS = m_packet->BEAT_INTERVAL;
					if(m_packet->IS_LIVE  == 1){
						LIVE = TRUE;
						call ReadTimer.startOneShot(READ_INTERVAL_MS);
				 	}else if(m_packet->IS_LIVE == 0 ) { 
				 		LIVE = FALSE;
					}
					printf("UPDATE_PATIENT_PACKET recieved");
					printfflush();
			
		
				}

		}
	
		if(len == sizeof(REQUEST_MSG)){
			printf("Request Received\n");
			call LogRead.seek(0);
		}
	


        return msgPtr;
    }


  

  event void ECGMsgSend.sendDone(message_t *m, error_t error){}
  
}
