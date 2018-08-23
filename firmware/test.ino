/* Minimum_Source*/
#include "stdio.h"
#include "string.h"

#define MAX_ID 20
#define MAX_BUFFER 100

/***************************************************************/
#define SERVO_INSTRUCTION 0x01
#define TORQUE_INSTRUCTION 0x02
#define LED_INSTRUCTION 0x03

#define POSITION_REQUEST 0x10
#define VOLTAGE_REQUEST 0x20
#define TEMPERATURE_REQUEST 0x30
#define LOAD_REQUEST 0x40
#define ID_POSITION_REQUEST 0x50
#define TRANSMIT_MODE 0xF0
#define READ_POSITION_MODE 0xF1

#define WRITE_REGISTER_BYTE 0x60
#define WRITE_REGISTER WORD 0x61
#define READ_REGISTER_BYTE_REQUEST 0x62
#define READ_REGISTER_WORD_REQUEST 0x63
/***************************************************************/

#define PRESENT_POSITION 0x24
#define PRESENT_VOLTAGE 0x2A
#define PRESENT_TEMPERATURE 0x2B
#define PRESENT_LOAD 0x28

#define TORQUE_ENABLE 0x18
#define LED_ENABLE 0x19

enum packet_status_t {idle, start_packet_1, start_packet_2, length_rxd, instruction_rxd, read_param, checksum};
enum transmit_mode_t {no_operation = 0x00, position_mode = 0x01, position_torque_mode = 0x02};
enum read_position_mode_t {position_one_shot = 0x00, position_stream = 0x01};

packet_status_t statusPacket = idle;
transmit_mode_t transmitMode = no_operation;
read_position_mode_t positionMode = position_stream;

char instructionBuffer = 0;
int dataLength = 0;
char dataBuffer[MAX_BUFFER];
int dataCount = 0;
char dataCheckSum = 0;

int indexToWrite=1;

int servoPosition[MAX_ID] = {512,512,512,512,512,512,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,512,512};
int servoPosition2[MAX_ID] = {512,512,512,512,512,512,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,512,512};
int servoReadPosition[MAX_ID] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
int servoTorque[MAX_ID];
int servoLed[MAX_ID] ;//= {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

char cservoReadPosition[2*MAX_ID];

int ledInstruction = 0;
int torqueInstruction = 0;

int voltageRequest = 0;
int temperatureRequest = 0;
int loadRequest = 0;
int positionOneShot = 0;

char voltageRead[MAX_ID];
char temperatureRead[MAX_ID];
int loadRead[MAX_ID];

char cloadRead[2*MAX_ID];

word syncPosition[40]=
{
  1,512,  //1
  2,512,  //3
  3,512,  //5
  4,512,  //7
  5,512,  //9
  6,512,  //11
  7,2048, //13
  8,2048, //15
  9,2048, //17
  10,2048,  //19
  11,2048,  //21
  12,2048,  //23
  13,2048,  //25
  14,2048,  //27
  15,2048,  //29
  16,2048,  //31
  17,2048,  //33
  18,2048,  //35
  19,512, //37
  20,512};  //39

Dynamixel Dxl(1);

void setup() {
  // put your setup code here, to run once:
  SerialUSB.attachInterrupt(usbInterrupt);
  pinMode(BOARD_LED_PIN,OUTPUT);
  Dxl.begin(3);
}

void usbInterrupt(byte* buffer, byte nCount){
//  SerialUSB.print("nCount : ");
//  SerialUSB.print((int)nCount);
//  SerialUSB.print(" data received : ");
//  for(int i=0; i<nCount; i++){
//    if(i==nCount-1)
//      SerialUSB.println((char)buffer[i]);
//    else
//      SerialUSB.print((char)buffer[i]);
//  }
  for(int i=0; i<nCount; i++)
    updateState((char)buffer[i]);
}

void loop() {
  // put your main code here, to run repeatedly: 
    toggleLED();
    switch(transmitMode){
      case position_torque_mode : {
        for(int i=0; i<20; i++){
          if(servoTorque[i]){
            Dxl.goalPosition(i+1,servoPosition[i]);
          }
          else 
            Dxl.writeByte(i+1,TORQUE_ENABLE,0);
        }
        break;
      }
      default : 
        transmitMode = no_operation;
        break;
    }
  
    switch(positionMode){
      case position_one_shot:{
        if(positionOneShot){
          positionOneShot = 0;
        }
        break;
      }
      case position_stream:{
        for(int i=0; i<20; i++){
          
          int servoReadP = Dxl.readWord(i+1,PRESENT_POSITION);
          servoReadPosition[i] = servoReadP;
          
          if(servoReadP == 65535){
            servoReadP = Dxl.readWord(i+1,PRESENT_POSITION);
            if(servoReadP == 65535){
              servoReadP = servoReadPosition[i];
            }
            else 
              servoReadPosition[i] = servoReadP;
          }
          else {
            servoReadPosition[i] = servoReadP;
          }
          char cservoReadPh = servoReadP>>8;
          char cservoReadPl = servoReadP&0xff;
          sendPosition(i+1,cservoReadPh,cservoReadPl);
        }
        break;
      }
      default:
        positionMode = position_one_shot;
        break;
    }
  
    if(ledInstruction){
      for(int i=0; i<20; i++){
        Dxl.writeByte(i+1,LED_ENABLE,servoLed[i]);
      }
      ledInstruction=0;
    }
    
    if(voltageRequest){
      for(int i=0; i<20; i++){
        voltageRead[i]=Dxl.readByte(i+1,PRESENT_VOLTAGE);
      }
      sendPacket(voltageRead,20,VOLTAGE_REQUEST);
      voltageRequest=0;
    }
  
    if(temperatureRequest){
      for(int i=0; i<20; i++){
        temperatureRead[i]=Dxl.readByte(i+1,PRESENT_TEMPERATURE);
      }
      sendPacket(temperatureRead,20,TEMPERATURE_REQUEST);
      temperatureRequest=0;
    }
  
    if(loadRequest){
      for(int i=0; i<20; i++){
        loadRead[i]=Dxl.readWord(i+1,PRESENT_LOAD);
        cloadRead[2*i]=loadRead[i]>>8;
        cloadRead[2*i+1]=loadRead[i]&0xff;
      }
      sendPacket(cloadRead,40,LOAD_REQUEST);
      loadRequest=0;
    }
    
  delay(1);
}

void updateState(char data){
  char rxdata = data;
  switch(statusPacket){
    case idle:{
    if(rxdata==0xff)
      statusPacket=start_packet_1;
    break;
    }
    case start_packet_1:{
      if(rxdata==0xff)
        statusPacket=start_packet_2;
      else statusPacket = idle;
      break;
    }
    case start_packet_2:{
      if(rxdata>0){
        dataLength=rxdata;
        statusPacket=length_rxd;
      }
      else statusPacket = idle;
      break;
    }
    case length_rxd:{
      instructionBuffer=rxdata;
      statusPacket=instruction_rxd;
      break;
    }
    case instruction_rxd:{
      if(dataLength>2){
        dataCount=0;
        dataBuffer[dataCount++] = rxdata;
        statusPacket = read_param;
      }
      else {
        dataCheckSum = rxdata;
        char temp=dataLength+instructionBuffer;
        for(int i=0; i<dataLength-2; i++)
        temp+=dataBuffer[i];
        temp = ~temp;
        int check=0;
        if(dataCheckSum==temp)
         check=1;
        else check=0;
        processRequest(instructionBuffer);
        statusPacket = idle;
      }
      break;
    }
    case read_param:{
      if(dataCount>=dataLength-2){
        dataCheckSum = rxdata;
        char temp=dataLength+instructionBuffer;
        for(int i=0; i<dataLength-2; i++)
        temp+=dataBuffer[i];
        temp = ~temp;
        int check=0;
        if(dataCheckSum==temp){
         check=1;
         processRequest(instructionBuffer);
        }
        else check=0;
        statusPacket = idle;      
      }
      else 
        dataBuffer[dataCount++] = rxdata;
      break;
    }
   default:
     SerialUSB.println("error");
     statusPacket=idle;
  }
}

void processRequest(char instruction){
//  SerialUSB.print("instruction : ");
//  printHex(instruction);
  switch(instruction){
    
    case SERVO_INSTRUCTION:{
      for(int i=0; i<20; i++){
        int temp = (dataBuffer[2*i]<<8) | dataBuffer[2*i+1];
        servoPosition[i] = temp;
        
        syncPosition[2*i+1] = temp;
      }
      Dxl.syncWrite(30,1,syncPosition,40);
      break;
    }
    
    case TORQUE_INSTRUCTION:{
      for(int i=0; i<20; i++){
        servoTorque[i] = dataBuffer[i];
      }
      break;
    }
    
    case LED_INSTRUCTION:{
      for(int i=0; i<20; i++){
        servoLed[i]=dataBuffer[i];
      }
      ledInstruction = 1;
      break;
    }
    
    case VOLTAGE_REQUEST:{
      voltageRequest = 1;
      break;
    }
      
    case TEMPERATURE_REQUEST:{
      temperatureRequest = 1;
      break;
    }
      
    case LOAD_REQUEST:{
      loadRequest = 1;
      break;
    }
      
    case TRANSMIT_MODE:{
      char temp = dataBuffer[0];
      if(temp==0x00){
        transmitMode = no_operation;
      }
      else if(temp==0x01){
        transmitMode = position_mode;
      }
      else if(temp==0x02){
        transmitMode = position_torque_mode;
      }
      else {
        transmitMode = no_operation;
      }
      break;
    }
      
    case READ_POSITION_MODE:{
      char temp = dataBuffer[0];
      if(temp == 0){
        positionMode = position_one_shot;
        positionOneShot = 0;
      }
      else if(temp == 1){
        positionMode = position_stream;
      }
      else {
        positionMode = position_one_shot;
      }
      break;
    }
		  
	case WRITE_REGISTER_BYTE:{
		int id = dataBuffer[0];
		char regs = dataBuffer[1];
		char value = dataBuffer[2];
		Dxl.writeByte(id,regs,value);
		break;
	}
	  case WRITE_REGISTER_WORD:{
		  int id = dataBuffer[0];
		  char regs = dataBuffer[1];
		  int value = (dataBuffer[2]<<8)+dataBuffer[3];
		  Dxl.writeWord(id,regs,value);			  
		  break;
	  }
	  case READ_REGISTER_BYTE_REQUEST:{
		  break;
	  }
	  case READ_REGISTER_WORD_REQUEST:{
		  break;
	  }
		  
    default:
      break;
  }
}

void sendPosition(int id, char high, char low){
  char packet[3];
  packet[0] = id; packet[1] = high; packet[2] = low;
  sendPacket(packet, 3, ID_POSITION_REQUEST);
}

void sendPacket(char*data, int ndata, char instruction){
  char packet[45];
  char checksum = ndata+2+instruction;
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = ndata+2;
  packet[3] = instruction;
  for(int i=0; i<ndata; i++){
    packet[i+4] = data[i];
    checksum+=data[i];
  }
  packet[ndata+4] = ~checksum;
  for(int i=0; i<ndata+5; i++){
    SerialUSB.print(packet[i]);
  }
}

void printHex(char data){
  char lower = data&0x0f;
  char higher = (data>>4);
//  SerialUSB.print("0x");
  switch(higher){
    case 0:
      SerialUSB.print("0");
      break;
    case 1:
      SerialUSB.print("1");
      break;
    case 2:
      SerialUSB.print("2");
      break;
    case 3:
      SerialUSB.print("3");
      break;
    case 4:
      SerialUSB.print("4");
      break;
    case 5:
      SerialUSB.print("5");
      break;
    case 6:
      SerialUSB.print("6");
      break;
    case 7:
      SerialUSB.print("7");
      break;
    case 8:
      SerialUSB.print("8");
      break;
    case 9:
      SerialUSB.print("9");
      break;
    case 10:
      SerialUSB.print("A");
      break;
    case 11:
      SerialUSB.print("B");
      break;
    case 12:
      SerialUSB.print("C");
      break;
    case 13:
      SerialUSB.print("D");
      break;
    case 14:
      SerialUSB.print("E");
      break;
    case 15:
      SerialUSB.print("F");
    default:
      break;
  }
  switch(lower){
    case 0:
      SerialUSB.print("0");
      break;
    case 1:
      SerialUSB.print("1");
      break;
    case 2:
      SerialUSB.print("2");
      break;
    case 3:
      SerialUSB.print("3");
      break;
    case 4:
      SerialUSB.print("4");
      break;
    case 5:
      SerialUSB.print("5");
      break;
    case 6:
      SerialUSB.print("6");
      break;
    case 7:
      SerialUSB.print("7");
      break;
    case 8:
      SerialUSB.print("8");
      break;
    case 9:
      SerialUSB.print("9");
      break;
    case 10:
      SerialUSB.print("A");
      break;
    case 11:
      SerialUSB.print("B");
      break;
    case 12:
      SerialUSB.print("C");
      break;
    case 13:
      SerialUSB.print("D");
      break;
    case 14:
      SerialUSB.print("E");
      break;
    case 15:
      SerialUSB.print("F");
    default:
      break;
  }
//  SerialUSB.print(" ");
}

