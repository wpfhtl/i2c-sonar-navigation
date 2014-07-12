/*
i2c_srf04 by Vitor Christo
vitor.christo@gmail.com
July 2014     V0.1
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 
 
 This read 5 sonar type SRF04, and sends it by I2C (left,right,front,back,alt)
 the idea is to navigate in restricted areas (indoors)
 and also help in the autolanding
 
*/


#include <Wire.h>
#include <EEPROM.h>
#include "sonar.h"
#include <PID_v1.h>
//#define debug_xy

/**********************************************************
*  PID int & Vars
***********************************************************/
//Define Variables we'll be connecting to
double SetpointAlt=100, InputAlt=100, outputAlt=0;
double SetpointX=100, InputX=100, outputX=0;
double SetpointY=100, InputY=100, outputY=0;
double Kp=1, Ki=0.05, Kd=0.25;

//Specify the links and initial tuning parameters
PID AltPID(&InputAlt, &outputAlt, &SetpointAlt, Kp, Ki, Kd, DIRECT);
PID XPID(&InputX, &outputX, &SetpointX, Kp, Ki, Kd, DIRECT);
PID YPID(&InputY, &outputY, &SetpointY, Kp, Ki, Kd, DIRECT);
//#define debug  // uncomment to debug
uint8_t SONAR_ADD=0x70;

const int pingPin_1= 10;  //strob
const int pingPinEcho_1 = 11; //echo

const int pingPin_2 = 2;  //strob
const int pingPinEcho_2 = 3; //echo

const int pingPin_3 = 4;  //strob
const int pingPinEcho_3 = 5; //echo

const int pingPin_4 = 6;  //strob
const int pingPinEcho_4 = 7; //echo

const int pingPin_5 = 8;  //strob
const int pingPinEcho_5 = 9; //echo

int reading = 0;
int unit=0;
int sonar=0;
// int sample=1;
uint16_t left=0;
uint16_t right=0;
uint16_t front=0;
uint16_t alt=0;
uint16_t rear=0;
bool altUp=false;
bool altDown=false;
bool yRequest=false;
bool xRequest=false;
bool altRequest=false;
bool altLock = false;
uint8_t minXyDist=100; 
bool frontFlag=false;
bool rearFlag=false;
bool leftFlag=false;
bool rightFlag=false;
uint16_t safeDist=100; // front/rear right/left safe distance to keep 
uint16_t safeAlt=100; // Alt safe distance to keep
unsigned long tolerance=10; // window tolerance percent
/************************************************************
*    to do 
*    - allow multi sample 
*    - Use only one pinPing pin
*************************************************************/


long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

void set_i2c_add(void){
   uint8_t data=Wire.read();
   uint8_t validate=Wire.read();
   if(validate==EEPROM_DAT_VALIDATE){
   		EEPROM.write(EEPROM_I2C_ADD,data);
   		EEPROM.write(EEPROM_ADD_VALIDATE,validate);
   }
  }
void receiveEvent(int size){
uint8_t command,reading,tmp1,tmp2;
uint16_t readDat;
	 if(size <= Wire.available())    // if two bytes were received
  		{
    		command = Wire.read();   // receive high byte (overwrites previous reading)
    		switch (command){
    		case 0: // get zero commands
    			reading = Wire.read();
    			switch (reading){
    				case 51:
    					unit=0;  // cm
    				break;
    				case 52:
    					unit=1;  // inc
    				break;
    				case 53:
    					unit=3;  // milis
    				break;
    				case 60:
    					sonar=1;
    				break;
    				case 61:
    					sonar=2;
    				break;
    				case 62:
    					sonar=3;
    				break;
    				case 63:
    					sonar=4;
    				break;
    				case 64:
    					sonar=5;
    				break;
    				case 70:
    					set_i2c_add();
    				break;
    				default:
    				 break;
    			
    				};	
    		case 1:	
    	 			// byte to set x,y tolerance 
    			while(Wire.available()){
    				tolerance=Wire.read();
    				if(tolerance < 10 && tolerance > 90) tolerance = 20;
    			}
    		break;
    		case 2: // request Y_PID fronte/rear
    			
    			yRequest=true;
    			
    			
    		break;
    		case 3: // request X_PID left/right
    			xRequest=true;

    		break;
    		case 4: // Set minimum altitud   		
    		  if (2 <= Wire.available())   // if two bytes were received
  				{
    				uint8_t tmp1=Wire.read();
    				uint8_t tmp2=Wire.read();
    						 readDat=tmp2 <<8;
    						 readDat |=tmp1;
    						 SetpointAlt=readDat;
				}

    		break;
    		case 5: // request altitud PID
    			altRequest=true;   			
    		break;
    		case 6: // sets the minimum x / y distance
    			while(Wire.available()){
    				minXyDist=Wire.read();
    				if(minXyDist < 50 || minXyDist > 255) minXyDist=100;
    			}
    		break;
    			
    		default:
    		break;
    	}
#ifdef debug
    		Serial.print("comand reading");   // print the reading
    		Serial.println(reading);
#endif debug 
  		}
}
void requestEvent(){
#ifdef debug
    		Serial.println("Event Request");   // print the reading
    		
#endif debug 

  
	uint16_t wdat;
	uint8_t dat;
 	if(sonar){
		if(sonar==1){ wdat=left; sonar=0;}
		if(sonar==2){wdat=right; sonar=0;}
		if(sonar==3){wdat=front; sonar=0;}// front
		if(sonar==4){wdat=alt;sonar=0;}
		if(sonar==5){wdat=rear;sonar=0;} // back
		Wire.write((uint8_t *)&wdat,2);
	}
	if(altRequest){
		altRequest=false;
		if (altUp && !altDown)
			dat=0;
		else
		if(altDown && !altUp)
			dat=1;
			else 
			if(altUp && altDown)
				dat=2;
			else dat=3; 
		Wire.write((uint8_t *)&dat,1);
		Wire.write((uint8_t *)&outputAlt,1);	
		}
		
/*		
	if(xRequest){ // leftflag sends 0
		xRequest=false;
		if(leftFlag && !rightFlag){
			dat=0;
			leftFlag=false;
			}
		else
			if(rightFlag && !leftFlag){  // right sends 1
				rightFlag=false;
				dat=1;
			} 
			else 
			if(leftFlag && rightFlag) dat=2; // locked 
			else 
			dat=3;  // out of range
		Wire.write((uint8_t *)&dat,1);
		Wire.write((uint8_t *)&outputX,1);					
	}	
*/

	if(xRequest){
		xRequest=0;
		wdat=outputX;
		wdat &=0xFF;
		if(leftFlag) 
				wdat |=0x100;
				else if(!rightFlag && !leftFlag)
					wdat |= 0x200;
#ifdef debug_xy
Serial.println("***********************************************");
Serial.println("*         Lef and Right                       *");
Serial.println("***********************************************");
Serial.print("dados enviados = ");
Serial.println(wdat);

#endif debug_xy		
		Wire.write((uint8_t *)&wdat,2);
	}


	if(yRequest){
		yRequest=0;
		wdat=outputY;
		wdat &=0xFF;
		if(rearFlag) 
			wdat |=0x100;
			else if(!rearFlag && !frontFlag)
					wdat |= 0x200;
		Wire.write((uint8_t *)&wdat,2);
#ifdef debug_xy
Serial.println("***********************************************");
Serial.println("*         Frente and Tras                     *");
Serial.println("***********************************************");
Serial.print("dados enviados = ");
Serial.println(wdat);

#endif debug_xy	
	}


/*

		
	if(yRequest ){ // leftflag sends 0
		yRequest=false;
		if(frontFlag && !rearFlag){
			dat=0;
			frontFlag=false;
			}
			else
				if(rearFlag && !frontFlag){  // right sends 1
					rearFlag=false;
					dat=1;
					}else 
						if(frontFlag && rearFlag ) dat=2; // locked 
						else dat=3; // aut of range
		uint16_t out=outputY;
		out = out << 8;
		out |=dat;
	
		Wire.write((uint8_t *)&out,2);					
	}	
			
	*/

#ifdef debug
  Serial.println(wdat);
#endif debug
}

void setup(void){
	  uint8_t eep_test = EEPROM.read(EEPROM_ADD_VALIDATE);
	  if(eep_test==EEPROM_DAT_VALIDATE)
	  	SONAR_ADD = EEPROM.read(EEPROM_I2C_ADD);
	  	Serial.begin(57600);
#ifdef debug
	Serial.begin(57600);
	Serial.print("I2c ADD = ");
	Serial.println(SONAR_ADD);
#endif debug
	XPID.SetOutputLimits(OUTPUT_LIMIT_MIN,OUTPUT_LIMIT_MAX );
  	XPID.SetMode(AUTOMATIC);
  	YPID.SetOutputLimits(OUTPUT_LIMIT_MIN,OUTPUT_LIMIT_MAX);
  	YPID.SetMode(AUTOMATIC);
  	AltPID.SetOutputLimits(OUTPUT_LIMIT_MIN,OUTPUT_LIMIT_MAX);
  	AltPID.SetMode(AUTOMATIC);
	Wire.begin(SONAR_ADD);                // join i2c bus with address #SONAR_ADD
  	Wire.onRequest(requestEvent); 		// register event	  
  	Wire.onReceive(receiveEvent);		// register event int bytes
	  // EEPROM.write(addr, val);
}

void loop(){
get_data();
uint16_t med;
med=(front+rear)/2; // front + rear indoor range found
tolerance=(front+rear)*tolerance/100;
if(yRequest){
	if(front && rear){
		if(front < (med + tolerance)){
			frontFlag=true;
			rearFlag=false;
			SetpointY=med;
			InputY=front;
			YPID.Compute();
#ifdef debug
	Serial.println(" Request Fw ");
#endif debug
		}else 
			if(rear < (med + tolerance)){
				frontFlag=false;
				rearFlag=true;
				SetpointY=med;
				InputY=rear;
				YPID.Compute();	
#ifdef debug
	Serial.println(" Request BACK ");
#endif debug		
		}else {
			frontFlag=true;
			rearFlag=true;
		}
	}else
		if(!front && (rear< med+ tolerance)){
			rearFlag=true;
			frontFlag=false;
			SetpointY=minXyDist;
			InputY=rear;
			YPID.Compute();
#ifdef debug
	Serial.println(" Request BACK front out of range ");
#endif debug
		}else
			if(!rear && (front < med + tolerance)){
				rearFlag=false;
				frontFlag=true;
				SetpointY=minXyDist;
				InputY=front;
				YPID.Compute();	
#ifdef debug
	Serial.println(" Request FRONT back out of range ");
#endif debug	
			}else {
				rearFlag=false;
				frontFlag=false;
				outputY=0;
#ifdef debug
	Serial.println(" FRONT / BACK LOCKED ");
#endif debug
			}



}
if(xRequest){
#ifdef debug
	Serial.println(" Request X  ");
#endif debug
	if(left && right){
		med=(left + right)/2;
		if(left < (med + tolerance)){
			leftFlag=true;
			rightFlag=false;
			SetpointX=med;
			InputX=left;
			XPID.Compute();
#ifdef debug
	Serial.println(" GO LEFT  ");
#endif debug
		}else 
			if(right < (med + tolerance)){
				rightFlag=true;
				leftFlag=false;
				SetpointX=med;
				InputX=right;
				XPID.Compute();
#ifdef debug
	Serial.println(" GO RIGHT  ");
#endif debug
			}else{
				rightFlag=true;
				leftFlag=true;
			}
	}else
		if(!left && (right < med+ tolerance)){
			rightFlag=true;
			leftFlag=false;
			SetpointX=minXyDist;
			InputX=right;
			XPID.Compute();
#ifdef debug
	Serial.println(" GO RIGHT left is aout of range  ");
#endif debug		
		
		}else
			if(!right && (left < med + tolerance)){
				rightFlag=false;
				leftFlag=true;
				SetpointX=minXyDist;
				InputX=left;
				XPID.Compute();
#ifdef debug
	Serial.println(" GO LEFT right is out of range  ");
#endif debug		
			}else {
				rearFlag=false;
				frontFlag=false;
				outputX=0;
#ifdef debug
	Serial.println(" Do NOTING it is locked!  ");
#endif debug
			}
}
tolerance=(safeAlt + tolerance )/100;
if(altRequest){
	if(alt){ 
		if(alt < (safeAlt + tolerance)){
			altUp=true;
			altDown=false;
			SetpointAlt=safeAlt;
			InputAlt=alt;
			AltPID.Compute();
		}
		else
			if(alt > (safeAlt + tolerance)){
				altUp=false;
				altDown=true;
				SetpointAlt=safeAlt;
				InputAlt=alt;
				AltPID.Compute();
			}
			else{
				altUp=true;
				altDown=true;
			} 
	}else{  // pulseIn timeout (set to 400 cm) returns 0 out of range
			altUp=false;
			altDown=true;
			SetpointAlt=safeAlt;
			InputAlt=alt;
			AltPID.Compute();
		}
}

}

void get_data(void){


long duration, inches, cm;
  pinMode(pingPin_1, OUTPUT);
  digitalWrite(pingPin_1, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_1, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_1, LOW);
  pinMode(pingPinEcho_1, INPUT);
  duration = pulseIn(pingPinEcho_1, HIGH,PULSE_TIMEOUT);	
  if(unit==0){ 
  		cm = microsecondsToCentimeters(duration);
    	left=cm;
   }
   if(unit==1){
    	inches = microsecondsToInches(duration);
    	left=inches;
   }
   
   if(unit==2)
    	left=duration;
   
    
  pinMode(pingPin_2, OUTPUT);
  digitalWrite(pingPin_2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_2, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_2, LOW);
  pinMode(pingPinEcho_2, INPUT);
  duration = pulseIn(pingPinEcho_2, HIGH,PULSE_TIMEOUT);	
  if(unit==0){ 
  		cm = microsecondsToCentimeters(duration);
    	right=cm;
   }
   if(unit==1){
    	inches = microsecondsToInches(duration);
    	right=inches;
   }
   if(unit==2)
    	right=duration;
   
   
  pinMode(pingPin_3, OUTPUT);
  digitalWrite(pingPin_3, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_3, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_3, LOW);
  pinMode(pingPinEcho_3, INPUT);
  duration = pulseIn(pingPinEcho_3, HIGH,PULSE_TIMEOUT);	
  if(unit==0){ 
  		cm = microsecondsToCentimeters(duration);
    	front=cm;
   }
   if(unit==1){
    	inches = microsecondsToInches(duration);
    	front=inches;
   }
  if(unit==2)
    	front=duration;
   
  
  pinMode(pingPin_4, OUTPUT);
  digitalWrite(pingPin_4, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_4, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_4, LOW);
  pinMode(pingPinEcho_4, INPUT);
  duration = pulseIn(pingPinEcho_4, HIGH,PULSE_TIMEOUT);	
  if(unit==0){ 
  		cm = microsecondsToCentimeters(duration);
    	alt=cm;
   }
   if(unit==1){
    	inches = microsecondsToInches(duration);
    	alt=inches;
   }
   if(unit==2)
    	alt=duration;
   
   
  pinMode(pingPin_5, OUTPUT);
  digitalWrite(pingPin_5, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_5, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_5, LOW);
  pinMode(pingPinEcho_5, INPUT);
  duration = pulseIn(pingPinEcho_5, HIGH,PULSE_TIMEOUT);	
  if(unit==0){ 
  		cm = microsecondsToCentimeters(duration);
    	rear=cm;
   }
   if(unit==1){
    	inches = microsecondsToInches(duration);
    	rear=inches;
   }
  if(unit==2)
  rear=duration;
  }
  
    
