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



  event void Boot.booted(){
    call RadioControl.start();
    ID = TOS_NODE_ID;
    call Notify.enable();
    call ReadTimer.startPeriodic(READ_INTERVAL_MS);
    printf("started\n");
    printfflush();

  }


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

  void sendMessage(ECG_DATA * buf){
	ECG_PACKET * m_packet = (ECG_PACKET*)(call Packet.getPayload(&msg, sizeof (ECG_PACKET)));
		m_packet->NODE_ID = ID;
		m_packet->D1 = buf->D1;
 		m_packet->D2 = buf->D2;
 		m_packet->D3 = buf->D3;
 		m_packet->D4 = buf->D4;
 		m_packet->D5 = buf->D5;
 		m_packet->D6 = buf->D6;
 		m_packet->D7 = buf->D7;
 		m_packet->TIME = buf->TIME;

	call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(ECG_PACKET)); 

  }


 event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
    printf("read done!\n");
    if ( (len == sizeof(ECG_DATA)) && (buf == &m_entry) ) {
       //call Send.send(&m_entry.msg, m_entry.len);
      printf("%i %i %i %i %i %i %i %i ", 		
		m_entry.D1 , 
 		m_entry.D2 ,
 		m_entry.D3 ,
 		m_entry.D4 ,
 		m_entry.D5 ,
 		m_entry.D6 ,
 		m_entry.D7 ,
 		m_entry.TIME);

      //sendMessage(buf);
      printfflush();
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


  event void ReadTimer.fired(){
	
	uint32_t currGlobTime = call GlobalTime.getLocalTime();
        call GlobalTime.getGlobalTime(&currGlobTime);


	if (!m_busy) {
		m_entry.D1 =1;
 		m_entry.D2 =1;
 		m_entry.D3 =1;
 		m_entry.D4 =1;
 		m_entry.D5 =1;
 		m_entry.D6 =1;
 		m_entry.D7 =1;
 		m_entry.TIME =currGlobTime;
 		
		
		m_busy = TRUE;
		if (call LogWrite.append(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
		    m_busy = FALSE;
		    printf("write error");
		    printfflush();
		}
        }




	/*RssiMsg * rssi_msg = (RssiMsg*)(call Packet.getPayload(&msg, sizeof (RssiMsg)));

	Sequence_Number = ++Sequence_Number % 1000;

	
	
	rssi_msg->NODE_ID = ID;
	rssi_msg->seq_num = Sequence_Number;

	if(rssi_msg->NODE_ID == 33){
		call Leds.led0Toggle();
	} else {
		call Leds.led1Toggle();
	}
	


	call RssiMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(RssiMsg));  */  
  }

  event void ECGMsgSend.sendDone(message_t *m, error_t error){}
}
