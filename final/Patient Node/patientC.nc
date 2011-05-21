#include "ECGHeader.h"
#include <Timer.h>
#include <UserButton.h>
#include "printf.h"

module patientC {
  uses{
	interface Boot;
	interface Leds;
	interface Timer<TMilli> as ReadTimer;
	interface Timer<TMilli> as HeartBeatTimer;
	  
	interface Packet;
	interface AMPacket;
        interface Receive;
	interface AMSend as ECGMsgSend;
	interface SplitControl as RadioControl;
	interface GlobalTime<TMilli>;

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
  // CHANGE THIS FOR REAL LIVE
  bool LIVE = TRUE;


  event void Boot.booted(){
    call RadioControl.start();
    ID = TOS_NODE_ID;
    call Notify.enable();
    call ReadTimer.startPeriodic(READ_INTERVAL_MS);
    printf("started\n");
    printfflush();

  }

 //Currently, on button press, read flash and send to base
 //Change this to be either timed or when flash getting full
  event void Notify.notify (button_state_t state) {
    if ( state == BUTTON_PRESSED) {
      call LogRead.seek(0);
    }
	seq = 0;
  }

  event void LogRead.seekDone(error_t err) {
    printf("wooo");
    if (call LogRead.read(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
        printf("error");
        printfflush();
    };
  }

  //Send message (this will currently loop, sending one message for each peice of data stored
  // Innefecient, need to ask about sending large messages
  void sendMessage(void * buf){

	ECG_DATA *ECGData = (ECG_PACKET*) buf;
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

  //Event fired after each read from the flash, (ie, each full struct of ECG data is read 
 event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
    printf("read done!\n");
    if ( (len == sizeof(ECG_DATA)) && (buf == &m_entry) ) {
       //call Send.send(&m_entry.msg, m_entry.len);

	
      /* CHANGE THIS TO SEND MESSAGE VIA RADIIO */
      
      sendMessage(buf);
      
      call Leds.led1On();
      call LogRead.read(&m_entry, sizeof(ECG_DATA));
    }
    else {
      if (call LogWrite.erase() != SUCCESS) {
	// Handle error.
      }
      call Leds.led0On();
    }
  }




event void LogWrite.eraseDone(error_t err) {
    if (err == SUCCESS) {
      m_busy = FALSE;
    }
    else {
      // Handle error.
    }
    call Leds.led0Off();
  }


  event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
    m_busy = FALSE;
    call Leds.led2Off();
  }



  event void LogWrite.syncDone(error_t err) {
  }





  event void RadioControl.startDone(error_t result){
    call ReadTimer.startPeriodic(READ_INTERVAL_MS);
    call HeartBeatTimer.startPeriodic(BEAT_INTERVAL_MS);
 }

  event void RadioControl.stopDone(error_t result){}

  event void HeartBeatTimer.fired(){
    BEAT_MSG * beat_msg = (BEAT_MSG*)(call Packet.getPayload(&msg, sizeof (BEAT_MSG)));
  	
    beat_msg->NODE_ID = ID;

    call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(BEAT_MSG));

  }


  ECG_DATA * readRawECGData(){
	ECG_DATA* dummyData;
			dummyData.D1 =1;
	 		dummyData.D2 =1;
	 		dummyData.D3 =1;
	 		dummyData.D4 =1;
	 		dummyData.D5 =1;
	 		dummyData.D6 =1;
	 		dummyData.D7 =1;
	 		dummyData.TIME =currGlobTime;
	return dummyData;


  }
  

  //Dummy data, should read from ecg. Reads ECG data, stores on flash.
  //Need to implement local checking to test for emrgency.
  event void ReadTimer.fired(){
	
	uint64_t currGlobTime = call GlobalTime.getLocalTime();
        call GlobalTime.getGlobalTime(&currGlobTime);
	
	if(LIVE == TRUE){
		if (!m_busy) {

	 		m_entry = readRawECGData();
		
			m_busy = TRUE;
			if (call LogWrite.append(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
			    m_busy = FALSE;
			    printf("write error");
			    printfflush();
			}
		}
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
	UPDATE_PATIENT_PACKET * packet;

	if(len == sizeof(UPDATE_PATIENT_PACKET)) {
	packet = (UPDATE_PATIENT_PACKET*)call Packet.getPayload(msgPtr, 
					sizeof(UPDATE_PATIENT_PACKET));

		if(packet ->NODE_ID == ID){
			READ_INTERVAL_MS = packet->READ_INTERVAL;
                        BEAT_INTERVAL_MS = packet->BEAT_INTERVAL;
			LIVE = packet->IS_LIVE;
			
			printf("UPDATE_PATIENT_PACKET recieved");
			printfflush();
			
		
		}

	}


        return msgPtr;
    }


  

  event void ECGMsgSend.sendDone(message_t *m, error_t error){}
}
