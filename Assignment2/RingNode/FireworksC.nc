#include "Fireworks.h"


module FireworksC
{
    uses
    {
        interface GlobalTime<TMilli>;
        interface TimeSyncInfo;
        interface Receive;
        interface AMSend;
        interface Packet;
		interface AMPacket;
        interface Leds;
        interface PacketTimeStamp<TMilli,uint32_t>;
        interface Boot;
        interface SplitControl as RadioControl;
		interface Timer<TMilli> as LedTimer;
    }
}

implementation
{
    message_t msg;
    bool locked = FALSE;
	uint16_t LOCAL_ID;
	uint16_t NODE_COUNT = 1;
	bool ledOn = FALSE;
	bool isBaseNode= FALSE;
	uint16_t nodeList[100];
	bool running = FALSE;


	// Funciton for regular nodes, tells basenode that this node is alive, and sends its real noe id
	void liveNode(){
		fireworks_node_msg* fnm = (fireworks_node_msg*)(call Packet.getPayload(&msg, 
				sizeof (fireworks_node_msg)));	

		fnm->real_node_id = TOS_NODE_ID;		

		//THIS SUCCEEDS
		call AMSend.send(AM_FIREWORKS_MSG, &msg, sizeof(fireworks_node_msg)) == SUCCESS;
			
	
	}



    event void Boot.booted() {
		uint16_t i;        
		call RadioControl.start();
			
		call Leds.led0Off();

		//If this is the base node (real node id0, set up)		
		if(TOS_NODE_ID == 0){
			isBaseNode = TRUE;
			LOCAL_ID = 0;
			NODE_COUNT = 1;
			for(i=0; i< 100; i++){
				nodeList[i] = 0;
			}
			signal LedTimer.fired();
			
		} else {
		//Otherwise signal to the basenode that this node alive
			liveNode();
			signal LedTimer.fired();
		}
    }

	//Calculate the next time that this node should blink
	uint32_t getNextOnTime(uint32_t currTime){
		uint32_t lapTime = currTime % (LED_PERIOD * NODE_COUNT);	
		return ((LED_PERIOD * NODE_COUNT) - lapTime + (LED_PERIOD * LOCAL_ID));
	}


	event void LedTimer.fired() {
		uint32_t currGlobTime;	
		if(ledOn == FALSE){
			call Leds.led0On();
			ledOn = TRUE;
			call LedTimer.startOneShot(LED_PERIOD);
	
		} else {
			//Turn led off, call basenode to say im still alive, calculate next on time
			call Leds.led0Off();
			liveNode();			
			currGlobTime = call GlobalTime.getLocalTime();
			call GlobalTime.getGlobalTime(&currGlobTime);
			ledOn = FALSE;
			call LedTimer.startOneShot(getNextOnTime(currGlobTime));
			


		}

		

	}


	
	//Send to all nodes the new number of nodes, when the nodes recieve this message, 
	//if the individual node is the new node added to the network, ie, the nodes TOS_NODE_ID = realID
	//then that nodes new local id = the current node count. This is designed so that nodes can enter 
 	// the network without haveing a sequential nodeid. eg current network is nodes 0,1,2 and then node 45
	// joins the network, node 45's local id will be set to 3, allowing it to blink in the correct position.
	void updateNodes(uint16_t realID){
			fireworks_update_msg * fum = (fireworks_update_msg*)(call Packet.getPayload(&msg, 
				sizeof (fireworks_update_msg)));	

		fum->id = realID;
		fum->node_count = NODE_COUNT;

		call AMSend.send(AM_FIREWORKS_MSG, &msg, sizeof(fireworks_update_msg));
	}

	// Base node recieves a message from a node. If that node is already in the netwrok, does nothing.
	// If node is a new node, finds that nodes position in the array, and stores its new local ID, which 
	// will be the new nodecount. This is setup for expansion, so that each node will have to update that 
	// it is still active, so that an inactive node will be removed form the network.
	void baseUpdate(	message_t* msgPtr, void* payload, uint8_t len ){

		fireworks_node_msg *fnm = (fireworks_node_msg*) payload;
		uint16_t realID = fnm->real_node_id;

		if(nodeList[realID] == 0){
			nodeList[realID] = NODE_COUNT;
			updateNodes(realID);			
			NODE_COUNT++;
		} 
		
		if(NODE_COUNT >1 ){
			running = TRUE;
		}

	}

	//When a regular node recieves a message form the basenode, if the message was meant 
	// just for this node, then set this nodes local id, otherwise, just update the current node count.

	void nodeUpdate(message_t* msgPtr, void* payload, uint8_t len ){

		uint32_t rxTimestamp;
		uint16_t to_id;
        if (!locked && call PacketTimeStamp.isValid(msgPtr)) {
            fireworks_update_msg* fum = (fireworks_update_msg*)call Packet.getPayload(msgPtr, 
					sizeof(fireworks_update_msg));
			to_id = fum->id;
			if(to_id == TOS_NODE_ID){
				LOCAL_ID = fum->node_count;
			}			
			NODE_COUNT = fum->node_count;


            rxTimestamp = call PacketTimeStamp.timestamp(msgPtr);
            call GlobalTime.local2Global(&rxTimestamp);
			
			running = TRUE;
            
        }

	}

	
	// recieves a message 
	// THIS NEVER PROCS!!!!! DONT KNOW WHY
    event message_t* Receive.receive(message_t* msgPtr, void* payload, uint8_t len)
    {
		call Leds.led2Toggle();
		
		if( isBaseNode == TRUE) {
			call Leds.led1Toggle();
			baseUpdate(msgPtr, payload, len );
		} else {
			call Leds.led1Toggle();
			nodeUpdate(msgPtr, payload, len );
		}


        return msgPtr;
    }

    event void AMSend.sendDone(message_t* ptr, error_t success) {\
        return;
    }


	event void RadioControl.startDone(error_t err){
		if (err == SUCCESS) {
		} else {
			call RadioControl.start();
		}
	}

    event void RadioControl.stopDone(error_t error){}
}