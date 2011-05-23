#include <Timer.h>
#include "../ECGHeader.h"
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
		//interface Timer<TMilli> as TimeKeeper;
		interface GlobalTime<TMilli>;		


   		interface Tcp;
		interface SplitControl as RadioControl;

		interface Packet;
		interface AMPacket;
		interface AMSend  as ECGMsgSend;
		interface SplitControl as AMControl;
		interface Receive;




	}





} implementation {



	bool busy = FALSE;	
  	bool timerStarted;	
  	
  	message_t r_msg;							


  	bool dataReady = FALSE;
  	bool inProgress = FALSE;
  	//ECG_PACKET m_entry;
  	
  	ECG_PACKET dataArray[50];
  	int dataCount = 0;
  	int position = 0;
  	int readPosition = 0;
  	EMER_MSG EmrMsg;
  	int heartBeat = 0;
  	
	
  	static char *http_okay = "HTTP/1.0 200 OK\r\n\r\n";
  	static int http_okay_len = 19;

  	enum {
    	S_IDLE,
    	S_CONNECTED,
    	S_REQUEST_PRE,
    	S_REQUEST,
    	S_HEADER,
    	S_BODY,
  	};

	enum {
		HTTP_GET,
		HTTP_POST,
	};

	//Sends the current global time to the server
	void sendTime(){

		char time[10];
		//char* r = time;
		char* reply;
		uint32_t currGlobTime = call GlobalTime.getLocalTime();
	
		call GlobalTime.getGlobalTime(&currGlobTime);
	
		ltoa(currGlobTime, time, 10);
					
		reply = (char *)malloc(strlen(time) * sizeof(char));
		memcpy(reply, time, strlen(time));
		
		
		printf("Time : %s", reply);
		printfflush();
		
		call Tcp.send(http_okay, http_okay_len);
		call Tcp.send(reply, strlen(time));
		call Tcp.close();
		free(reply);
	}

	//Check which nodes have sent a heartbeat and send to server
	void sendBeats(){
		char reply[10];
		char buf[15];
		
		call Tcp.send(http_okay, http_okay_len);
		
		if(heartBeat != 0){
			itoa(heartBeat, reply, 10);
			strcpy(buf, reply);
			strcat(buf, "/");
			strcat(buf, "/");
			strcat(buf, "\n");
			
			heartBeat = 0;
			
			printf(" Beat Message : %s\n", buf);
			printfflush();
			
			call Tcp.send(buf, strlen(buf) );
		}

		call Tcp.close();
	}

	//Check which nodes have sent an emergency and send to server
	void sendEmer(){
		char id[10];
		char type[10];
		char time[15];
		char buf[30];
		
		call Tcp.send(http_okay, http_okay_len);
		
		printf("GETTING EMERGENCY\n");
		printfflush();
		
		if(EmrMsg.TYPE != 0){
			itoa(EmrMsg.NODE_ID, id, 10);
			itoa(EmrMsg.TYPE, type, 10);
			ltoa(EmrMsg.TIME, time, 10);
			
			strcpy(buf, id);
			strcat(buf, "/");
			strcat(buf, type);
			strcat(buf, "/");
			strcat(buf, time);
			strcat(buf, "/");
			strcat(buf, "/");
			strcat(buf, "\n");
			
			printf("EMRGENCY MESSAGE : %s\n", buf);
			printfflush();
			
			call Tcp.send(buf, strlen(buf) );
			
		}
		
		
		

		
		call Tcp.close();

	}


	//Return request from patient node, with data to be returned to the server
	void send_TCP_ECG_Message(){
		char buf[100];
		char id[10] ;
		char D1[10] ;
		char D2[10] ;
		char D3[10] ;
		char D4[10] ;
		char D5[10] ;
		char D6[10] ;
		char D7[10] ;
		char TIME[10] ;
		int i = 0;
		int pos = 0;

		
		printf("Count : %i \n",  dataCount);
		printfflush();
		
		pos = readPosition;
		
		for(i = pos; i < pos +5; i++){
				 
			 ECG_PACKET ECGMsg = dataArray[i]; 
			 
			 //printf("BLAH BLAH BLAH\n\n\n");
			 //printfflush();			 
				 
			 itoa(ECGMsg.NODE_ID, id, 10);
			 itoa(ECGMsg.D1, D1, 10);
			 itoa(ECGMsg.D2, D2, 10);
			 itoa(ECGMsg.D3, D3, 10);
			 itoa(ECGMsg.D4, D4, 10);
			 itoa(ECGMsg.D5, D5, 10);
			 itoa(ECGMsg.D6, D6, 10);
			 itoa(ECGMsg.D7, D7, 10);
			 ltoa(ECGMsg.TIME, TIME, 10);


			 strcpy(buf, id);
			 strcat(buf, "/");
			 strcat(buf, D1);
			 strcat(buf, "/");
			 strcat(buf, D2);
			 strcat(buf, "/");
			 strcat(buf, D3);
			 strcat(buf, "/");
			 strcat(buf, D4);
			 strcat(buf, "/");
			 strcat(buf, D5);
			 strcat(buf, "/");
			 strcat(buf, D6);
			 strcat(buf, "/");
			 strcat(buf, D7);
			 strcat(buf, "/");
			 strcat(buf, TIME);
			 strcat(buf, "/");
			 strcat(buf, "/");
			 strcat(buf, "\n");
			 
		 
			  printf("BUFFER : %s, size : %i \n", buf, strlen(buf));
			 printfflush();
			 call Tcp.send(buf, strlen(buf));
			
			 buf[0] = '\0';
			 
			readPosition++;
			if(readPosition >= dataCount) break;
			 
			 //printf("BLAH BLAH BLAH\n\n\n");
			 //printfflush();
		}
		
		printf("Read Position : %i\n", readPosition);
		
		if(readPosition >= dataCount){
			position = 0;
			readPosition = 0;
			dataCount = 0;
			dataReady = FALSE;
			inProgress = FALSE;
			call Tcp.send("done\n", 5);
		}
		


	
		
	}
	
		//Send radio signal to specified node, requesting its stored data
	void sendRequest(char *request){
    
		char id[4] ;
		REQUEST_MSG *m_packet = (REQUEST_MSG*)(call Packet.getPayload(&r_msg, sizeof (REQUEST_MSG)));
		
		printf("Progress : %i   dataReady : %i \n", inProgress, dataReady);
		
		if( inProgress == FALSE) {
			inProgress = TRUE;
			printf("SEND REQUEST\n");
			printfflush();
	
	
			strncpy(id, request+11, 3);
			m_packet->NODE_ID = atoi(id);
			
			
			call Tcp.send(http_okay, http_okay_len);
			call Tcp.send("started\n", 8);
			call Tcp.close();
			
			

			call ECGMsgSend.send(AM_BROADCAST_ADDR, &r_msg, sizeof(REQUEST_MSG)); 
			return;
		} 
		
		if(dataReady == TRUE && inProgress == TRUE) {
			printf("SHOULD NOW SEND DATA TO SERVER\n");
			printfflush();
			call Tcp.send(http_okay, http_okay_len);
			send_TCP_ECG_Message();
			call Tcp.close();

			return;
		}  
		
		if(dataReady == FALSE && inProgress == TRUE) {
			printf("STILL GETTING DATA FORM MOTE\n");
			printfflush();
			call Tcp.send(http_okay, http_okay_len);
			call Tcp.send("wait\n", 5);
			call Tcp.close();
			
			return;
		}

	}
	


	//At the end of the patient data request, close the connection
	void sendFinishedRequest(){
		printf("CLOSING CONNECTION\n");
		printfflush();
		
		dataReady = TRUE;

	}

	//Extract data from the reqest, and send radio signal to specified node 
	//to set up its mode
	void setPatientNode(char *request){
		char id[4] ;
		char beats[6];
		char read[6];
		char live[2];
	

		UPDATE_PATIENT_PACKET *m_packet = (UPDATE_PATIENT_PACKET*)(call Packet.getPayload(&r_msg, sizeof (UPDATE_PATIENT_PACKET)));


		printf("UPDATE PATIENT : %s \n", request);
		printfflush();	

		strncpy(id, request+5, 3);
		id[3] = '\0';
		strncpy(beats, request+9, 5);
		beats[5] = '\0';
		strncpy(read, request+15, 5);
		read[5] = '\0';
		strncpy(live, request+21, 1);
		live[1] = '\0';

		 
		//printf("Data before sub ID:%s  BEAT:%s READ:%s LIVE%s \n",id , beats, read, live);
		//printfflush();
		 
		m_packet->NODE_ID = atoi(id);
		m_packet->BEAT_INTERVAL =  atol(beats);
		m_packet->READ_INTERVAL = atol(read);
	 	m_packet->IS_LIVE =  atoi(live);
	 	m_packet->MIN_HR =  0;
	 	m_packet->MAX_HR =  0;
	 	
	 	
	 	//printf("ID : %i\n", m_packet->NODE_ID);
	 	//printf("BEAT:%u\n", m_packet->BEAT_INTERVAL);
	 	//printf("READ:%u\n", m_packet->READ_INTERVAL);
	 	//printf("LIVE:%i\n", m_packet->IS_LIVE);
	 	
		printfflush();
	 	
	
		call Tcp.send(http_okay, http_okay_len);
		call Tcp.close();
	
		call ECGMsgSend.send(AM_BROADCAST_ADDR, &r_msg, sizeof(UPDATE_PATIENT_PACKET)); 
	}
	
	
	//Extract data from the reqest, and send radio signal to specified node 
	//to set up its mode
	void setPatientLimits(char *request){
		char id[4] ;
		char hi[4];
		char low[4];

	

		UPDATE_PATIENT_PACKET *m_packet = (UPDATE_PATIENT_PACKET*)(call Packet.getPayload(&r_msg, sizeof (UPDATE_PATIENT_PACKET)));


		printf("UPDATE PATIENT : %s \n", request);
		printfflush();	

		strncpy(id, request+9, 3);
		id[3] = '\0';
		strncpy(hi, request+13, 3);
		hi[3] = '\0';
		strncpy(low, request+17, 5);
		low[3] = '\0';

		 
		m_packet->NODE_ID = atoi(id);
		m_packet->BEAT_INTERVAL =  0;
		m_packet->READ_INTERVAL = 0;
	 	m_packet->IS_LIVE =  3;
	 	m_packet->MIN_HR =  atoi(low);
	 	m_packet->MAX_HR =  atoi(hi);

	 	
		printfflush();
	 	
	
		call Tcp.send(http_okay, http_okay_len);
		call Tcp.close();
	
		call ECGMsgSend.send(AM_BROADCAST_ADDR, &r_msg, sizeof(UPDATE_PATIENT_PACKET)); 
	}


	//Send an alarm to the nurse nodes
	void alarm(uint8_t type){
		ALARM *m_packet = (ALARM*)(call Packet.getPayload(&r_msg, sizeof (ALARM)));
	
		printf("SEND ALARM\n");
		printfflush();
	
		
		m_packet->TYPE = type;
		call Tcp.send(http_okay, http_okay_len);
		call Tcp.close();
	
		printf("Type ->%i\n",m_packet->TYPE );
		printfflush();
	
	
		call ECGMsgSend.send(AM_BROADCAST_ADDR, &r_msg, sizeof(ALARM)); 
	}

	//Process the incoming request from the tcp connection, and call the correct responce function
	void process_request(int verb, char *request, int len) {
		char id[2];
		
		if (len >= 5 &&
			request[0] == '/' &&
			request[1] == 's' &&
			request[2] == 'e' &&
			request[3] == 'n' &&
			request[4] == 'd' )
		{
			alarm(1);
		}
	   	if (len >= 9 &&
			request[0] == '/' &&
			request[1] == 'g' &&
			request[2] == 'e' &&
			request[3] == 't' &&
			request[4] == '/') 
		{
			//Get global time
			if(request[5] == 't' &&
			  	request[6] == 'i' &&
			  	request[7] == 'm' &&
			  	request[8] == 'e')
			{
				sendTime();

			}
			
			//Get current beats
			if(request[5] == 'b' &&
			  	request[6] == 'e' &&
			  	request[7] == 'a' &&
			  	request[8] == 't')
			{
				sendBeats();
			}
			
			//Get current emergencies
			if(request[5] == 'e' &&
			  	request[6] == 'm' &&
			  	request[7] == 'e' &&
			  	request[8] == 'r')
			{
				sendEmer();
			}
			
			//Request data from a node
			if(len >= 13 &&
				request[5] == 'd' &&
			  	request[6] == 'a' &&
			  	request[7] == 't' &&
			  	request[8] == 'a' &&
				request[9] == '/')
			{
				sendRequest(request);
			}
		}
		
		//Set a patient nodes mode
	   	if (len >= 22 &&
			request[0]  == '/' &&
			request[1]  == 's' &&
			request[2]  == 'e' &&
			request[3]  == 't' &&
			request[4]  == '/' &&
			request[8]  == '/' &&
			request[14] == '/' &&
			request[20] == '/') 
		{
		
			setPatientNode(request);
			
		
		}
		
		if (len >= 19 && 
			request[0]  == '/' &&
			request[1]  == 's' &&
			request[2]  == 'e' &&
			request[3]  == 't' &&
			request[4]  == '/' &&
			request[8]  == '/' &&
			request[12] == '/' &&
			request[16] == '/') 
		{
		
			setPatientLimits(request);
			
		
		}
		



	}


/////////////////// handle tcp connection ////////////////////////

  	int http_state;
  	int req_verb;
  	char request_buf[150], *request;
  	char tcp_buf[500];



  	event bool Tcp.accept(struct sockaddr_in6 *from, 
                            void **tx_buf, int *tx_buf_len) {
    	if (http_state == S_IDLE) {
      		http_state = S_CONNECTED;
      		*tx_buf = tcp_buf;
      		*tx_buf_len = 500;
      		return TRUE;
    	}
    	printfUART("rejecting connection\n");
    	return FALSE;
 	}
 	 
  	event void Tcp.connectDone(error_t e) {
    
  	}
  
  
  	event void Tcp.recv(void *payload, uint16_t len) {
    	static int crlf_pos;
    	char *msg = payload;
    	switch (http_state) {
    	case S_CONNECTED:
      		crlf_pos = 0;
      		request = request_buf;
      		if (len < 3) {
        		call Tcp.close();
        		return;
     		}
      		if (msg[0] == 'G') {
        		req_verb = HTTP_GET;
        		msg += 3;
        		len -= 3;
      		}
      		
      		http_state = S_REQUEST_PRE;
      		
    	case S_REQUEST_PRE:
      		while (len > 0 && *msg == ' ') {
        		len--; msg++;
      		}
      		if (len == 0) break;
      		http_state = S_REQUEST;
      		
    	case S_REQUEST:
      		while (len > 0 && *msg != ' ') {
        		*request++ = *msg++;
        		len--;
      		}
      		
      		if (len == 0) break;
      		*request++ = '\0';
      		http_state = S_HEADER;
      		
    	case S_HEADER:
      		while (len > 0) {
        		switch (crlf_pos) {
        			case 0:
        			case 2:
          				if (*msg == '\r') crlf_pos ++;
          				else if (*msg == '\n') crlf_pos += 2;
          				else crlf_pos = 0;
          				break;
        			case 1:
        			case 3:
          				if (*msg == '\n') crlf_pos ++;
          				else crlf_pos = 0;
          				break;
        		}
        		len--; msg++;
        		// if crlf == 2, we just finished a header line.  you know.  fyi.
        		if (crlf_pos == 4) {
          			http_state = S_BODY;
          			process_request(req_verb, request_buf, request - request_buf - 1);
          			break;
        		} 
      		}
      		
    		if (crlf_pos < 4) break;

    	case S_BODY:
      // len might be zero here... just a note.
    	default:
      		call Tcp.close();
    	}
  	}

  	event void Tcp.closed(error_t e) {
    	call Leds.led2Toggle();

    	call Tcp.bind(80);
    	http_state = S_IDLE;
  	}

  	event void Tcp.acked() {

  	}
  	
  	event void RadioControl.startDone(error_t e) {}

  	event void RadioControl.stopDone(error_t e) {}
		
	//////////////////////////////

	event void Boot.booted() {
		call AMControl.start();
		call RadioControl.start();

	    http_state = S_IDLE;
	    call Tcp.bind(80);
		
		printf("Booted\n");
		printfflush();
		EmrMsg.TYPE = 4;

				
	}


	void updateNodes(void *data, uint16_t len){

    	call ECGMsgSend.send(AM_BROADCAST_ADDR, &data, len);
	}


///////////////////// ///////////////////////////////////


 


	void beatReceived(BEAT_MSG* beatMsg){
         
        heartBeat = beatMsg->NODE_ID;
        
        //call Status.sendto(&route_dest, &beatMsg, sizeof(beatMsg));
 		printf("Beat Recieved From : %i\n", beatMsg->NODE_ID);
        printfflush();


	}





////////////////////////////////////////////////////////////





///////////////////// Radio /////////////////////////





	//Handle incoming radio messages (from patient nodes)
	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    	
		BEAT_MSG *beatMsg; 
        ECG_PACKET *ECGMsg;
		EMER_MSG *AlarmMsg;
		
		if(len == sizeof(BEAT_MSG)){	    
			beatMsg = (BEAT_MSG*) payload;
			beatReceived(beatMsg);

		}
		
		if(len == sizeof(ECG_PACKET)){
			ECGMsg = (ECG_PACKET*) payload;
	
			
			dataArray[dataCount] = *ECGMsg;
			position++;
			position = ((position % 50) == 0)? 0 : position;
			if(dataCount < 50) dataCount++;
			printf("COUNT : %i \n", dataCount);
        	printfflush();
			//send_TCP_ECG_Message(ECGMsg);
			

		}
		
		
		if(len == sizeof(REQUEST_DONE_MSG)){
			//printf("FINSHED SENDING PATIENT DATA \n");
        	//printfflush();	
        	sendFinishedRequest();
		}
		
		
		if(len == sizeof(EMER_MSG)){
			AlarmMsg = (EMER_MSG*) payload;

			
			
			
			if(AlarmMsg->TYPE == 0){
				EmrMsg.TYPE = 0;
				alarm(0);
			}else {
				EmrMsg.NODE_ID = AlarmMsg->NODE_ID;
				EmrMsg.TYPE = AlarmMsg->TYPE;
				EmrMsg.TIME = AlarmMsg->TIME;
				
				printf("ALARM Msg: %i %i %u \n",AlarmMsg->NODE_ID, AlarmMsg->TYPE, AlarmMsg->TIME  );
				printf("EMRMsg: %i %i %u \n",EmrMsg.NODE_ID, EmrMsg.TYPE , EmrMsg.TIME  );
				
				
				alarm(2);
			}

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
	
		if(error != SUCCESS){
			printf("Failed to send\n");
		} else {
			printf("Sent\n");
		}
		printfflush();
	}

	

  	





////////////////////////////////////////////////////////////////////


}
