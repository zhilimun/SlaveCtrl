/*
 * The code is originally developed by Shengqi Jian
 * The code is updated by Zhi Li starting from 14/03/2017
 */

#include "Mega_Slave.h"
#include <SPI.h>
#include "mcp_can.h"
#include <AccelStepper.h>
#include <stdio.h>
#include <avr/iom2560.h>
#include <Arduino.h>

const int Proximity_Top = 22;
const int Proximity_Bottom = 23;
const int Proximity_Head = 26;
const int Proximity_Tail = 27;

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
AccelStepper stepper_blade_L(1, 4, 24);    // direction Digital 2 (CW), pulses Digital 3 (CLK)
AccelStepper stepper_Z_L (1, 5, 25);

//Include the message for the MTS sensor
unsigned char stm_start[2] = {1, 1};
unsigned char stm_stop[2] = {2, 1};

unsigned long int ConvertString2Int(String Magnet_ps);
float CAN_MTS_Read();
void CAN_Send(char data0,char data1,char data2,char data3,char data4,char data5,char data6,char data7);
unsigned int Flag_CAN_Read();
String Convert_Buff_To_String(unsigned char buf[8]);
int Proximity_Start_Detection(void);

void Lift_Caliper_Tool(AccelStepper stepper_Z_L);
void Drop_Caliper_Tool(AccelStepper stepper_Z_L);

#define encoder0PinA    19
#define encoder0PinB    18

#define interruptPin 2

unsigned int encoder0Pos = 0;
unsigned int encoder1Pos = 0;
long currentTime;
long loopTime1, loopTime2;

// This vector stores the position data of the caliper tool
unsigned int positionlist[5]={50,0,0,0,0};//position 0, 1,2,3,4

// Check if an emergency happens during the operation --> stop everyting
void checkTermin()
{
    String Receive_CAN;

    unsigned int canId;
    unsigned char len = 0;
    unsigned char buf[8];

	// Check the received CAN message
	CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    canId = CAN.getCanId();

    if(canId==128){
		char str[23];
		unsigned char * pin = buf;
		const char * hex = "0123456789ABCDEF";
		char * pout = str;

		for(; pin < buf+sizeof(buf); pout+=3, pin++){
			pout[0] = hex[(*pin>>4) & 0xF];
			pout[1] = hex[ *pin     & 0xF];
			pout[2] = ':';
		}
		pout[-1] = 0;
		Receive_CAN = String(str);

    	if(Receive_CAN=="02:04:06:00:00:00:00:00"){
    		stepper_blade_L.stop();
    		stepper_Z_L.stop();
    		termin();
    	}
    }

}



void doEncoder()
{
	 if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
	    encoder0Pos++;
	  } else {
	    encoder0Pos--;
	  }
}

int moveToPos(unsigned char caseNo, unsigned char stopcrit)
{
	int turnstep;
	int Position_float=0;
    // Used for encoder
    int cpms; 	//counts per 100 miliseconds
    unsigned int start_cpms=LOW;

	switch(caseNo){
	case 1://Approach the chain from left side
		turnstep=2000;
		break;
	case 2://Approach the chain from right side
		turnstep=-2000;
		break;
	}

    //Approach the chain with full speed (use the encoder to measure the rotational speed)
    currentTime = millis();
    loopTime1 = currentTime;
    loopTime2 = currentTime;
    encoder1Pos=encoder0Pos;

    while(1){

    	checkTermin();

        stepper_blade_L.move(turnstep);
        stepper_blade_L.run();

        // The motor needs some time to accelerate, so we have to start checking the speed after this acceleration process
        currentTime = millis();
        if(currentTime >= (loopTime1+5000)) //5 s
        {
        	start_cpms=HIGH;
        	//Serial.println("5s have passed");
        }
        if(start_cpms)
        {
			if(currentTime >= (loopTime2 + 25)){
				cpms=encoder0Pos-encoder1Pos;  //Count per 100 mili second
				loopTime2 = currentTime;
				encoder1Pos=encoder0Pos;
				Serial.print(cpms);
				Serial.println("\n\r");
				if(abs(cpms)<stopcrit)
				{
					start_cpms=LOW;
					break;
				}
			}
        }
    }
    for(int i=0; i<10; i++){
    	Position_float=CAN_MTS_Read(); delay(10);
    }

    return Position_float;
}

void setup()
{
    delay(1000);
	pinMode(encoder0PinA,INPUT);
	digitalWrite(encoder0PinA, HIGH);
	pinMode(encoder0PinB,INPUT);
	digitalWrite(encoder0PinB, HIGH);
	attachInterrupt(5, doEncoder, CHANGE);		//attachInterrupt(4, encoder_z, CHANGE);        //  INT4 --> Pin 19

	// Add the interrupt pin, when sensing change go to "termin" function
    //pinMode(interruptPin, INPUT);
    //digitalWrite(interruptPin, HIGH);
    //attachInterrupt(0, termin, CHANGE);			// INT0 --> Pin2

	//noInterrupts();  	  	  	  	  	  		//attachInterrupt(5, encoder_z, CHANGE);        //  INT5 --> Pin 18
	delay(1000);


	// The PWM frequency is important for the controlled stepper motor speed
	// Timer 0, 62K PWM frequency -- PIN 4 for the vertical motor
    int myEraser = 7;           // this is 111 in binary and is used as an eraser
    TCCR0B &= ~myEraser;   		// this operation (AND plus NOT),  set the three bits in TCCR0B to 0
    int myPrescaler = 2;        // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.
    TCCR0B |= myPrescaler;  	//this operation (OR), replaces the last three bits in TCCR0B with our new value 011

    // Timer 3, 31K PWM frequency -- PIN 5 for the vertical motor
    TCCR3B &= ~myEraser;
    TCCR3B |= myPrescaler;

    pinMode(Proximity_Head, INPUT);
    pinMode(Proximity_Tail, INPUT);
    pinMode(Proximity_Top, INPUT);
    pinMode(Proximity_Bottom, INPUT);

    // Serial communication to Laptop
    Serial.begin(115200);

    //CAN bus initialization
    while (CAN_OK != CAN.begin(CAN_1000KBPS))
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");


    // Horizontal motor
    stepper_blade_L.setMaxSpeed(150);
    stepper_blade_L.setAcceleration(30);
    stepper_blade_L.setCurrentPosition(0);
    // Vertical motor
    stepper_Z_L.setMaxSpeed(120);
    stepper_Z_L.setAcceleration(30);
    stepper_Z_L.setCurrentPosition(0);

    // Start the MTS sensor
    CAN.sendMsgBuf(0x00, 0, 2, stm_start);



}

//Declare reset function at address 0
void(* resetFunc) (void) = 0;

void loop()
{
    int Direction_status=0;
    unsigned int case_number=0;
    unsigned char len = 0;
    unsigned char buf[8];
    //unsigned char send_buff[8];

    float Position_float=0;
    float Position_float_old=0;
    float Position_float_new=0;

    //CAN related
    String Receive_CAN;
    String Receive_Master_Command="";
    String Magnet_position_string="";
    int Master_command_flag=0;
    int Flag_from_CAN_Read=0;
    unsigned int canId;

    // Measurement variables
    int position_val=0;
    unsigned char measureCnt;
    unsigned char measureTimes=1;

    //while(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    while(Master_command_flag==0)
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        canId = CAN.getCanId();

        char str[23];
        unsigned char * pin = buf;
        const char * hex = "0123456789ABCDEF";
        char * pout = str;
        String Result_String;
        for(; pin < buf+sizeof(buf); pout+=3, pin++){
            pout[0] = hex[(*pin>>4) & 0xF];
            pout[1] = hex[ *pin     & 0xF];
            pout[2] = ':';
        }
        pout[-1] = 0;
        Receive_CAN = String(str);

        if (canId==128){ // Message from Master Controller ID 0x0080 = 128
            Serial.print("Received Master Command 1: ");  Serial.println(Receive_CAN);
            Receive_Master_Command=Receive_CAN;
            Master_command_flag=1;
            break;
        }
        else if(canId==256){  // Message from MTS sensor ID 0x0100 = 256
            Master_command_flag=0;
        }
        else
        {
        	Serial.print("Received Data 2:"); Serial.println(Receive_CAN);
        }

    }

    Serial.println("Succeed Jump Out");
    if(Receive_Master_Command=="02:04:06:01:02:03:04:05"){ // Tool Caliper Initialization
        case_number=1;
    }
    if(Receive_Master_Command=="02:04:06:00:00:01:00:00"){ // Tool Caliper Position 1
        case_number=2;
    }
    if(Receive_Master_Command=="02:04:06:00:00:02:00:00"){ // Tool Caliper Position 2
        case_number=3;
    }
    if(Receive_Master_Command=="02:04:06:00:00:03:00:00"){ // Tool Caliper Position 3
        case_number=4;
    }
    if(Receive_Master_Command=="02:04:06:00:00:04:00:00"){ // Tool Caliper Position 4
        case_number=5;
    }
    if(Receive_Master_Command=="02:04:06:01:00:00:00:01"){ // Start Proximity Detection
        case_number=6;
    }
    if(Receive_Master_Command=="02:04:06:00:00:00:00:00"){ // Termination
        case_number=7;
    }
    if(Receive_Master_Command=="02:04:06:01:01:01:01:01"){ // Reboot
        case_number=8;
    }
    if(Receive_Master_Command=="02:04:06:01:02:03:04:06"){ // Measure all together
        case_number=9;
    }

    switch(case_number){
    case 1://Initialization
        Flag_from_CAN_Read=0;
        Lift_Caliper_Tool(stepper_Z_L);

        Serial.println("Blade Move Back To End"); // Blade move to back end
        for(int i=0; i<10; i++){
            Position_float=CAN_MTS_Read(); delay(10);
        }

        while(Position_float>100){
            Position_float=CAN_MTS_Read();

            stepper_blade_L.move(1000);
            stepper_blade_L.run();

            //stepper_blade_L.runSpeed();
        }
        stepper_blade_L.stop();
        stepper_blade_L.setCurrentPosition(0);
        Serial.println("Initial Point Reached!");

        //stepper_blade_L.setMaxSpeed(150);

        //Record the MTS measurement into the position list (an average of the 10 times measurements)
        for(int i=0; i<10; i++){
			position_val=CAN_MTS_Read(); delay(10);
		}
        positionlist[0]=position_val;
        position_val=0;

        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,6,7,8,9,10); // Finish Initial
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        Serial.println("Master Received");
        break;

    case 2://Measure position 1
        Flag_from_CAN_Read=0;
        Position_float_old=0;
        Position_float_new=CAN_MTS_Read();
        Position_float=Position_float_new-Position_float_old;
        Drop_Caliper_Tool(stepper_Z_L);

        Serial.println("Blade Moves to #1"); // Blade move to position #2

        // Approach the chain with the max speed
        for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
        {
        	position_val=moveToPos(2,5);//direction 1 - move right, 2 - move left, stop criteria
        	// Move back a little bit
			//stepper_blade_L.move(6000);
			//while(stepper_blade_L.run());
        }

		positionlist[1]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[1]);
        stepper_blade_L.stop();

        Serial.println("Position #1 Reached!"); // Finish position #1
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,1); // #1 Reached
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        Serial.println("Master Received");
        // Move back a little bit
		stepper_blade_L.move(6000);
		while(stepper_blade_L.run());
        break;
    case 3://Measure position 2
        Flag_from_CAN_Read=0;
        Position_float=CAN_MTS_Read();

        // Move back a little bit before moving up
        //stepper_blade_L.move(6000);
		//while(stepper_blade_L.run());

		Lift_Caliper_Tool(stepper_Z_L);

        //Drive the blade into the gap
        while(Position_float<positionlist[1]+100){
                   Position_float=CAN_MTS_Read();
                   stepper_blade_L.move(-1000);
                   stepper_blade_L.run();
        }

        Drop_Caliper_Tool(stepper_Z_L);

        Serial.println("Blade Moves to #2"); // Blade move to position #2

        // Approach the chain with the max speed
        for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
        {
        	position_val=moveToPos(1,5);//direction 1 - move right, 2 - move left, stop criteria
        	// Move back a little bit
			//stepper_blade_L.move(-6000);
			//while(stepper_blade_L.run());
        }

		positionlist[2]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[2]);
        stepper_blade_L.stop();

        Serial.println("Position #2 Reached!"); // Finish position #2
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,2); // Position #2 Reached
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        Serial.println("Master Received");

        // Move back a little
		stepper_blade_L.move(-6000);
		while(stepper_blade_L.run());
        break;

    case 4:
        Flag_from_CAN_Read=0;
        Position_float=CAN_MTS_Read();

        Serial.println("Blade Moves to #3"); // Blade move to position #3
        // Approach the chain with the max speed
        for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
        {
        	position_val=moveToPos(2,5);//direction 1 - move right, 2 - move left, stop criteria
        	// Move back a little bit
			//stepper_blade_L.move(6000);
			//while(stepper_blade_L.run());
        }

		positionlist[3]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[3]);
        stepper_blade_L.stop();


        Serial.println("Position #3 Reached!"); // Finish position #3
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,3); // Position #3 Reached
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        Serial.println("Master Received");

        // Move back a little
		stepper_blade_L.move(6000);
		while(stepper_blade_L.run());
        break;

    case 5:
        Flag_from_CAN_Read=0;
        Position_float=CAN_MTS_Read();

        // Move back a little bit before moving up
		//stepper_blade_L.move(6000);
		//while(stepper_blade_L.run());

        Lift_Caliper_Tool(stepper_Z_L);

        Serial.println("Blade Moves to #4"); // Blade move to position #4
        //Drive the blade into the gap
	    while(Position_float<positionlist[3]+100){
				  Position_float=CAN_MTS_Read();
				  stepper_blade_L.move(-1000);
				  stepper_blade_L.run();

	   }

	   Drop_Caliper_Tool(stepper_Z_L);

	   Serial.println("Blade Moves to #2"); // Blade move to position #2

	   // Approach the chain with the max speed
	   for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
	   {
			position_val+=moveToPos(1,5);//direction 1 - move right, 2 - move left, stop criteria
			// Move back a little bit
			//stepper_blade_L.move(-6000);
			//while(stepper_blade_L.run());
	   }

		positionlist[4]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[4]);
		stepper_blade_L.stop();

        Serial.println("Position #4 Reached!"); // Finish position #4
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,4); // Position #4 Reached
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
		//Move back a little bit
        stepper_blade_L.move(-6000);
		while(stepper_blade_L.run());

        Serial.println("Master Received");
        break;

    case 6:
        Direction_status=Proximity_Start_Detection();

        do{
            if(Direction_status==1){
                CAN_Send(1,3,5,10,9,8,7,1); // Proximity Forward Detected Finish
            }
            else if(Direction_status==-1){
                CAN_Send(1,3,5,10,9,8,7,0); // Proximity Backward Detected Finish
            }
            delayMicroseconds(2500);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        while(Flag_from_CAN_Read!=2);
        Serial.println("Master Received");
        break;
    case 7:
        Serial.println("Slave Termination Detected Jumped In");
        /*
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,0); // Proximity Head Detected
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
         */
        /*
        while(1){
            CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
            canId = CAN.getCanId();
            char str[23];
            unsigned char * pin = buf;
            const char * hex = "0123456789ABCDEF";
            char * pout = str;
            String Result_String;
            for(; pin < buf+sizeof(buf); pout+=3, pin++){
                pout[0] = hex[(*pin>>4) & 0xF];
                pout[1] = hex[ *pin     & 0xF];
                pout[2] = ':';
            }
            pout[-1] = 0;
            Result_String = String(str);

            if((canId==128)&&(Result_String=="02:04:06:01:01:01:01:01")){
                Serial.println("Slave Reboot from Termination"); delay(10);
                resetFunc(); }//call reset
            else{
                //Serial.println(Result_String);
                //CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
                //canId = CAN.getCanId();
            }
        }
        Serial.println("Master Received");
        */
        break;

    case 8:
        Serial.println("Slave Reboot Detected Jumped In");
        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,1,1,1,1,1); // Reboot Confirmed
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        resetFunc();
        break;
    case 9://Measure position 1-4 together
        Flag_from_CAN_Read=0;
        Position_float_old=0;
        Position_float_new=CAN_MTS_Read();
        Position_float=Position_float_new-Position_float_old;

        /******************** Position 1 *******************************/
        Drop_Caliper_Tool(stepper_Z_L);
        Serial.println("Blade Moves to #1"); // Blade move to position #2
        // Approach the chain with the max speed
        for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
        {
        	position_val=moveToPos(2,5);//direction 1 - move right, 2 - move left, stop criteria
        	// Move back a little bit
			stepper_blade_L.move(6000);
			while(stepper_blade_L.run());
        }
		positionlist[1]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[1]);
        stepper_blade_L.stop();
        Serial.println("Position #1 Reached!"); // Finish position #1

        /******************** Position 2 *******************************/
        Lift_Caliper_Tool(stepper_Z_L);
		//Drive the blade into the gap
		while(Position_float<positionlist[1]+150){
				   Position_float=CAN_MTS_Read();
				   stepper_blade_L.move(-1000);
				   stepper_blade_L.run();
		}
		Drop_Caliper_Tool(stepper_Z_L);
		Serial.println("Blade Moves to #2"); // Blade move to position #2
		// Approach the chain with the max speed
		for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
		{
			position_val=moveToPos(1,5);//direction 1 - move right, 2 - move left, stop criteria
			// Move back a little bit
			stepper_blade_L.move(-6000);
			while(stepper_blade_L.run());
		}
		positionlist[2]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[2]);
		stepper_blade_L.stop();
		Serial.println("Position #2 Reached!"); // Finish position #2

		/******************** Position 3 *******************************/
		Serial.println("Blade Moves to #3"); // Blade move to position #3
		// Approach the chain with the max speed
		for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
		{
			position_val=moveToPos(2,5);//direction 1 - move right, 2 - move left, stop criteria
			// Move back a little bit
			stepper_blade_L.move(6000);
			while(stepper_blade_L.run());
		}
		positionlist[3]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[3]);
		stepper_blade_L.stop();
		Serial.println("Position #3 Reached!"); // Finish position #3

		/******************** Position 4 *******************************/
        Lift_Caliper_Tool(stepper_Z_L);
        Serial.println("Blade Moves to #4"); // Blade move to position #4
        //Drive the blade into the gap
	    while(Position_float<positionlist[3]+150){
				  Position_float=CAN_MTS_Read();
				  stepper_blade_L.move(-1000);
				  stepper_blade_L.run();
	   }
	   Drop_Caliper_Tool(stepper_Z_L);
	   Serial.println("Blade Moves to #2"); // Blade move to position #2
	   // Approach the chain with the max speed
	   for(measureCnt=0;measureCnt<measureTimes;measureCnt++)
	   {
		position_val+=moveToPos(1,5);//direction 1 - move right, 2 - move left, stop criteria
		// Move back a little bit
		stepper_blade_L.move(-6000);
		while(stepper_blade_L.run());
	   }
		positionlist[4]=position_val/measureTimes;
		position_val=0;
		Serial.println("Position is:");
		Serial.print(positionlist[4]);
		stepper_blade_L.stop();
        Serial.println("Position #4 Reached!"); // Finish position #4


        while(Flag_from_CAN_Read!=2){
            CAN_Send(1,3,5,0,0,0,0,5); // #1 Reached
            //Serial.println("Master Not Received");
            delay(1);
            Flag_from_CAN_Read=Flag_CAN_Read();
            if(Flag_from_CAN_Read==2){
                break;
            }
        }
        Serial.println("Master Received");

        break;
    default:
        Receive_Master_Command="";delay(1);
        break;
    }
    CAN.clearMsg();

}

void termin()
{
	Serial.println("EEnter interrupt");delay(10);
	resetFunc();
}


void Lift_Caliper_Tool(AccelStepper stepper_Z_L){
    // Z axis move up -15350 if minus  (Counter Clock Wise)
	while(digitalRead(Proximity_Top)!=HIGH){

		checkTermin();

        stepper_Z_L.move(-2000);
        stepper_Z_L.run();
		//stepper_Z_L.setSpeed(-150);
		//stepper_Z_L.runSpeed();
    }
    stepper_Z_L.stop();
    delay(2000);
}

void Drop_Caliper_Tool(AccelStepper stepper_Z_L){

     // Z axis move up -15350 if minus  (Counter Clock Wise)
    while(digitalRead(Proximity_Bottom)!=HIGH){
    	checkTermin();

        stepper_Z_L.move(2000);
        stepper_Z_L.run();
    }
    stepper_Z_L.stop();
    delay(2000);
}

unsigned long int ConvertString2Int(String Magnet_ps){
    char Digits[7];
    unsigned long int Digits_int[6];
    unsigned long int Result;
    Magnet_ps.toCharArray(Digits,7,0);
    for(int i=0; i<6; i++){
        if((47<(int(Digits[i])))&&((int(Digits[i]))<58)){  // 47< x <58
            Digits_int[i]=int(Digits[i])-48;
        }
        else{
            Digits_int[i]=int(Digits[i])-65+10;
        }
    }
    Result=(Digits_int[0]*(1048576)+Digits_int[1]*65536+Digits_int[2]*4096+
            Digits_int[3]*256+Digits_int[4]*16+Digits_int[5]);
    return Result;
}
float CAN_MTS_Read(){
    float Position_float=0;
    unsigned char len = 0;
    unsigned char buf[8];
    String Receive_CAN;
    String Receive_Master_Command="";
    String Magnet_position_string="";

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        unsigned int canId = CAN.getCanId();
        char str[23];
        unsigned char * pin = buf;
        const char * hex = "0123456789ABCDEF";
        char * pout = str;
        for(; pin < buf+sizeof(buf); pout+=3, pin++){
            pout[0] = hex[(*pin>>4) & 0xF];
            pout[1] = hex[ *pin     & 0xF];
            pout[2] = ':';
        }
        pout[-1] = 0;
        Receive_CAN = String(str);

        if(canId==256){  // Message from MTS sensor ID 0x0100 = 256
            //Serial.println("-----------------------------");
            //Serial.print("Get data from ID: ");
            //Serial.println(canId);//, HEX);
            //Serial.println(Receive_CAN);Serial.println("\n\r");
            Magnet_position_string=Receive_CAN.substring(6,8)+Receive_CAN.substring(9,11)+Receive_CAN.substring(12,14);
            Position_float=ConvertString2Int(Magnet_position_string)*0.005;
            //Serial.print("Position :  "); Serial.println(Position_float);Serial.println("\n\r");
        }
        if(canId==128){
        	if(Receive_CAN=="02:04:06:00:00:00:00:00"){
				stepper_blade_L.stop();
				stepper_Z_L.stop();
				termin();
			}
        }
    }
    return Position_float;
}
void CAN_Send(char data0,char data1,char data2,char data3,char data4,char data5,char data6,char data7){
    //unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    stmp[0]=data0;stmp[1]=data1;stmp[2]=data2;stmp[3]=data3;
    stmp[4]=data4;stmp[5]=data5;stmp[6]=data6;stmp[7]=data7;
    CAN.sendMsgBuf(0x90, 0, 8, stmp);
}

unsigned int Flag_CAN_Read(){
    unsigned char len = 0;
    unsigned char buf[8];
    String Receive_CAN;
    unsigned int result_flag=0;
    String Receive_Master_Command;

    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
    char str[23];
    unsigned char * pin = buf;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    for(; pin < buf+sizeof(buf); pout+=3, pin++){
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
        pout[2] = ':';
    }
    pout[-1] = 0;
    Receive_CAN = String(str);
    if(canId==256){  // Message from MTS sensor ID 0x0100 = 256
        //Serial.println("-----------------------------");
        //Serial.print("Get data from ID: ");
        //Serial.println(canId);//, HEX);
        //Serial.println(Receive);
        //Magnet_position_string=Receive_CAN.substring(6,8)+Receive_CAN.substring(9,11)+Receive_CAN.substring(12,14);
        //Position_float=ConvertString2Int(Magnet_position_string)*0.005;
        //Serial.print("Position :  "); Serial.println(Position_float);
        result_flag=0;
    }
    else if (canId==128){ // Message from Master Controller ID 0x0080 = 128
        Serial.println("Received Master Command:");
        Serial.println(Receive_CAN);
        Receive_Master_Command=Receive_CAN;
        result_flag=1;
        if (Receive_Master_Command=="01:09:09:01:00:05:01:05"){
            Serial.println("Trapped");
            result_flag=2;
        }
    }
    return result_flag;
}

String Convert_Buff_To_String(unsigned char buf[8]){
    char str[23];
    unsigned char * pin = buf;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    String Result_String;
    for(; pin < buf+sizeof(buf); pout+=3, pin++){
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
        pout[2] = ':';
    }
    pout[-1] = 0;
    Result_String = String(str);
    return Result_String;
}

int Proximity_Start_Detection(void){
    boolean Proximity_Finish_Detection=false;
    int Direction_Status=0;
    boolean Forward_status=false;
    boolean Backward_status=false;
    boolean old_head_status=digitalRead(Proximity_Head);
    boolean old_tail_status=digitalRead(Proximity_Tail);
    boolean new_head_status=digitalRead(Proximity_Head);
    boolean new_tail_status=digitalRead(Proximity_Tail);
    Serial.println("Slave Proximity Program Jumped In");
    while((new_head_status==old_head_status)&&(new_tail_status==old_tail_status)){
        new_head_status=digitalRead(Proximity_Head);
        new_tail_status=digitalRead(Proximity_Tail);
        if ((new_head_status==!old_head_status)&&(new_tail_status==old_tail_status)){
            Forward_status=true; Backward_status=false;
            Direction_Status=1;
            break;
        }
        else if((new_head_status==old_head_status)&&(new_tail_status==!old_tail_status)){
            Forward_status=false; Backward_status=true;
            Direction_Status=-1;
            break;
        }
        else{}
    }
    Serial.print("Forward_status:  ");Serial.print(Forward_status);
    Serial.print("   Backward_status:  ");Serial.println(Backward_status);
    Serial.print("Result Status:  ");Serial.println(Direction_Status);
    while(Proximity_Finish_Detection==false){
        new_head_status=digitalRead(Proximity_Head);
        new_tail_status=digitalRead(Proximity_Tail);
        if((new_head_status==HIGH)&&(new_tail_status==HIGH)){
            delay(1);
            new_head_status=digitalRead(Proximity_Head);
            new_tail_status=digitalRead(Proximity_Tail);
            if((new_head_status==HIGH)&&(new_tail_status==HIGH)){
                Proximity_Finish_Detection=true;
            }
            else{Proximity_Finish_Detection=false;}
        }
        else{
            Proximity_Finish_Detection=false;
        }
    }
    Serial.println("Proximity Program Detected Finish");
    return Direction_Status;
}
