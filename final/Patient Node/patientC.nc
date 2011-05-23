///////////////////////////////////////
// CONFIGURATION PARAMETERS

#define MOVEMENT_THRESHOLD        200
#define ADC_BUFFER_SIZE           1000

///////////////////////////////////////

#include "../ECGHeader.h"
#include <Timer.h>
#include <UserButton.h>
#include "printf.h"
#include "Msp430Adc12.h"

module patientC {
	provides {
			interface AdcConfigure<const msp430adc12_channel_config_t*> as VoltageConfigure;
	}
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

		// ecg timers
		interface Read<uint16_t> as VoltageRead;

		interface Timer<TMilli> as MovementDelayTimer;
		interface Timer<TMilli> as AlarmTimer;
		interface Timer<TMilli> as SampleTimer;
		interface Timer<TMilli> as HRTimer;
		interface Timer<TMilli> as RTimer;



		interface LogRead;
		interface LogWrite;

		interface Notify<button_state_t>;


	}
} implementation {

	const msp430adc12_channel_config_t config = {
			/*inch: SUPPLY_VOLTAGE_HALF_CHANNEL,*/
		inch: INPUT_CHANNEL_A0,
			sref: REFERENCE_VREFplus_AVss,
			ref2_5v: REFVOLT_LEVEL_1_5,
			adc12ssel: SHT_SOURCE_ACLK,
			adc12div: SHT_CLOCK_DIV_1,
			sht: SAMPLE_HOLD_4_CYCLES,
			sampcon_ssel: SAMPCON_SOURCE_SMCLK,
			sampcon_id: SAMPCON_CLOCK_DIV_1
		};

		// send emergency message prototype
		void sendEmergency(uint8_t type);

	// ECG Variables

		uint8_t init = 0;
		uint8_t r_timer_ok = 0; // windowed r wave detection
		uint8_t r_period = 0; // adaptive r wave detection
		uint16_t r_threshold = 2750; // ADC threshold for r wave detection
		uint8_t r_flag = 0; // are we currently in an r wave ?
		uint16_t beat_interval = 0;
		uint16_t ms_count = 0;

		uint8_t movement_error = 0; // is the patient moving ?
		uint16_t movement_delay = 500; // how long should we wait before trying again ?

		uint16_t alarm_interval = 5000; // number of ms before alarm goes off

		uint16_t sample_interval = 3; // sample every n ms

		uint16_t adc_buffer[ADC_BUFFER_SIZE]; 
		uint16_t adc_buffer_index = 0;
		uint16_t i;

		uint16_t max_hr = 120;
		uint16_t min_hr = 50;

	// radio variables

		message_t msg;

		uint16_t ID;
		uint32_t BEAT_INTERVAL_MS = 15000;
		uint32_t READ_INTERVAL_MS = 1500;

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

			// start ECG timers
			call Leds.led0Off();
			call Leds.led1Off();
			call Leds.led2Off();
			call RTimer.startOneShot(MOVEMENT_THRESHOLD);
			call SampleTimer.startPeriodic(sample_interval);
			call HRTimer.startPeriodic(1);
		}
		
/////////////////////////////////////
// ECG code
/////////////////////////////////////		
		void restartAlarmTimer()
		{
			call AlarmTimer.stop();
			call AlarmTimer.startPeriodic(alarm_interval);
		}

		event void VoltageRead.readDone( error_t result, uint16_t val )
		{
			if (result == SUCCESS){
				if (movement_error == 0) {
					if ((val > r_threshold) && (r_flag == 0)) {
						if ((r_timer_ok == 1) || (init == 0)) {
							if (init == 0) {
								beat_interval = ms_count;
							} else {
								beat_interval = beat_interval*0.6 + 0.4*ms_count;
							}
							
							ms_count = 0;
							// r wave detected
							r_flag = 1;
							restartAlarmTimer();
							init = 1; // intialised
							call Leds.led0On();
							call Leds.led1Off(); // no error
						} else {
							call Leds.led1On(); // movement error
							movement_error = 1;
							call AlarmTimer.stop();
							call MovementDelayTimer.startOneShot(movement_delay);
						}
						r_timer_ok = 0;
						if (movement_error == 0) call RTimer.startOneShot(MOVEMENT_THRESHOLD);

					} else {
						if (val < r_threshold) r_flag = 0;
						call Leds.led0Off();
					}

				}
				if (adc_buffer_index < ADC_BUFFER_SIZE) {
					adc_buffer[adc_buffer_index++] = val;
				} else {

					call SampleTimer.stop();
					call AlarmTimer.stop();

					adc_buffer_index = 0;
					call SampleTimer.startPeriodic(sample_interval);
					restartAlarmTimer();

				}
				//printf(" ");
				//printfflush();
			}
			
		}

		async command const msp430adc12_channel_config_t* VoltageConfigure.getConfiguration()
		{
			return &config; // must not be changed
		}

		event void SampleTimer.fired()
		{
			call VoltageRead.read();
		}

		event void RTimer.fired() 
		{
			r_timer_ok = 1;
		}

		event void MovementDelayTimer.fired()
		{
			movement_error = 0;
			restartAlarmTimer();
			ms_count = 0;
		}

		event void AlarmTimer.fired()
		{
			call Leds.led2On();
			sendEmergency(3);
		}
		
		event void HRTimer.fired() 
		{
			ms_count++;
		}

////////////////////////////////////////////////////
// RADIO CODE
////////////////////////////////////////////////////


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
			call LogWrite.erase();
		}



	//Event fired after each read from the flash, (ie, each full struct of ECG data is read 
		event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
			printf("read done!\n");
			if ( (len == sizeof(ECG_DATA)) && (buf == &m_entry) ) {
			//call Send.send(&m_entry.msg, m_entry.len);


			/* CHANGE THIS TO SEND MESSAGE VIA RADIIO */

				sendMessage(buf);

				//call Leds.led1On();
				call LogRead.read(&m_entry, sizeof(ECG_DATA));

			} else {
				if (call LogWrite.erase() != SUCCESS) {
			// Handle error.
				}



					//call Leds.led1Off();
					//call Leds.led0On();
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

			//call Leds.led0Off();



			printf("Request Done \n");
			printfflush();

			recDoneMsg->NODE_ID = ID;
			recDoneMsg->TIME = currGlobTime;


			call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(REQUEST_DONE_MSG));


		}


		event void LogWrite.appendDone(void* buf, storage_len_t len, 
		bool recordsLost, error_t err) {
			m_busy = FALSE;
			//call Leds.led2Off();
		}



		event void LogWrite.syncDone(error_t err) {
		}





		event void RadioControl.startDone(error_t result){

			call HeartBeatTimer.startOneShot(BEAT_INTERVAL_MS);
			sendEmergency(0);
			
			
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

			printf("D1 : %u \n",60000/beat_interval );
			
			if( (60000/beat_interval) > max_hr ) sendEmergency(1);
			
			if( (60000/beat_interval) < min_hr ) sendEmergency(2);
			
			
			printfflush();
			m_entry.D1 = 60000/beat_interval; // heart rate
			m_entry.D2 =1;
			m_entry.D3 =1;
			m_entry.D4 =1;
			m_entry.D5 =1;
			m_entry.D6 =1;
			m_entry.D7 =1;
			m_entry.TIME =currGlobTime;



		}


	//Dummy data, should read from ecg. Reads ECG data, stores on flash.
	//Need to implement local checking to test for emrgency.
		event void ReadTimer.fired(){

			
			if(LIVE == TRUE){
			printf("READ RAW\n");
			printfflush();
			
				if (!m_busy) {

					readRawECGData();

					m_busy = TRUE;
					if (call LogWrite.append(&m_entry, sizeof(ECG_DATA)) != SUCCESS) {
						m_busy = FALSE;
						printf("write error");
						printfflush();
					} 
				}
				m_busy = FALSE;
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

			printf("ID : %i\n", m_packet->NODE_ID);
	 		printf("BEAT:%u\n", m_packet->BEAT_INTERVAL);
	 		printf("READ:%u\n", m_packet->READ_INTERVAL);
	 		printf("LIVE:%i\n", m_packet->IS_LIVE);
	 		printf("HI:%i\n", m_packet->MAX_HR);
	 		printf("LOW:%i\n", m_packet->MIN_HR);
	 		
	 		
			printfflush();

			if(m_packet ->NODE_ID == TOS_NODE_ID){
				if(m_packet->READ_INTERVAL != 0) READ_INTERVAL_MS = m_packet->READ_INTERVAL;
				if(m_packet->BEAT_INTERVAL != 0) BEAT_INTERVAL_MS = m_packet->BEAT_INTERVAL;
				if(m_packet->MAX_HR != 0) max_hr = m_packet->MAX_HR;
				if(m_packet->MIN_HR != 0) min_hr = m_packet->MIN_HR;

				if(m_packet->IS_LIVE  == 1){
					LIVE = TRUE;
					call ReadTimer.startOneShot(READ_INTERVAL_MS);
				}else if(m_packet->IS_LIVE == 0 ) { 
					LIVE = FALSE;
				}
				printf("Beat interval : %u\n", m_packet->BEAT_INTERVAL);
				
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
