#include "src/ClockHands.h"

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>


ClockHands Hands(19, 18, 21, 20, 15, 14, 17, 16, 400); 

byte ip[] = {172, 16, 0, 78};
byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x69, 0xEC};

//customisation: Artnet SubnetID + UniverseID (Defaluts to Universe 1)
byte SubnetID = {1};
byte UniverseID = {2};
short select_universe= ((SubnetID*16)+UniverseID);



//buffers
const int MAX_BUFFER_UDP=530;
char packetBuffer[MAX_BUFFER_UDP]; //buffer to store incoming data
short channelCount;
//byte buffer_channel_arduino[512]; //buffer to store filetered DMX data



// art net parameters
unsigned int localPort = 6454;      // artnet UDP port is by default 6454
const byte art_net_header_size=17;
const short max_packet_size=530;
char ArtNetHead[8]="Art-Net";
char OpHbyteReceive=0;
char OpLbyteReceive=0;
//short is_artnet_version_1=0;
//short is_artnet_version_2=0;
//short seq_artnet=0;
//short artnet_physical=0;
short incoming_universe=0;
boolean match_artnet=1;
short Opcode=0;
EthernetUDP Udp;

unsigned long DMXtime;

byte DMX[6] = {0,0,0,0,0,0};
byte lastDMX[6] = {0,0,0,0,0,0};

bool sleepState = true;

int DMXled = 1;
//---------------------------------------------------------------------------------------
//Setup Function
void setup()
{
  Serial.begin(38400);
  Hands.setSleep(false);
  sleepState = false;
  Hands.Calibrate();
  
  //reset WIZ850
  resetWiznet();

  Serial.println("start ethernet");
  //setup ethernet and udp socket
  Ethernet.begin(mac);
  Udp.begin(localPort);
  Serial.println("start ethernet complete");

  //setup status LEDs
  pinMode(DMXled, OUTPUT);
  digitalWrite(DMXled, 0);

  DMXtime = millis();
  
}

//----------------------------------------------------------------------------------------
//Helper Functions

short bytes_to_short_f(byte h, byte l){
  return( ((h << 8) & 0xff00) | (l & 0x00FF) );
}

//Reset WIZ850
void resetWiznet(){
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(50);
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(9, HIGH);
  delay(50);
}

//Get Artnet Data
int getArtnet(){
  int packetSize = Udp.parsePacket();

  //check packet size to see if it is in the right range
  if(packetSize>art_net_header_size) {

    
    //read the data into the buffer
    Udp.read(packetBuffer,MAX_BUFFER_UDP);
    
    //read header
    match_artnet=1;
    for (int i=0;i<7;i++) {
      //if not corresponding, this is not an artnet packet, so we stop reading
      if(char(packetBuffer[i])!=ArtNetHead[i]) {
        match_artnet=0;break;
      } 
    }

    if(match_artnet==1) {
      //operator code enables to know wich type of message Art-Net it is
      Opcode=bytes_to_short_f(packetBuffer[9],packetBuffer[8]);

      //if its DMX data we will read it now
      //DMX opcode is 0x5000
      if(Opcode==0x5000) {

        
        //read incoming universe
         incoming_universe = bytes_to_short_f(packetBuffer[15],packetBuffer[14]);

         //read channel count
         channelCount = bytes_to_short_f(packetBuffer[16],packetBuffer[17]);
          

         //if it is selected universe DMX will be read
         if(incoming_universe==select_universe) {
          


          //return the universe that was received
          return incoming_universe;
        }
      }
    }
  }
  return -1;
}



int channelOffset;
bool timeChange;
bool rotateChange;
int speed;
bool direction;
bool mode;
void loop()
{
//   Serial.println("loop");
  if(getArtnet()==select_universe)
  {
    digitalWrite(DMXled, 1);
    DMXtime = millis();
    if(sleepState){
      Hands.setSleep(false);
      sleepState = false;
      Hands.Calibrate();
    }
    
    for(int i=0;i< 6;i++)
    {
      DMX[i]= byte(packetBuffer[i+art_net_header_size+1]);
    }

    //see if a time or rotage change was requested
    timeChange = 0;
    for(int j=0;j<2;j++){
      if(DMX[j]!=lastDMX[j]){timeChange = 1;}
    }
    rotateChange = 0;
    for(int j=4;j<6;j++){
      if(DMX[j]!=lastDMX[j]){rotateChange = 1;}
    }

    if(timeChange){
      switch(DMX[2]){
        case 0:
          direction = 1;
          mode = 0;
          break;
        case 1:
          direction = 1;
          mode = 1;
          break;
        case 2:
          direction = 0;
          mode = 0;
          break;
        case 3:
          direction = 0;
          mode = 1;
          break;
      }
      speed = map(DMX[3],0,255,0,500);
      Hands.MoveToTime(DMX[0], DMX[1], mode, speed, direction);
    }else if(rotateChange & !timeChange){
      speed = map(DMX[5],0,255,0,500);
      switch(DMX[4]){
        case 0:
          Hands.stop();
          break;
        case 1:
          Hands.RotateRealTime(1);
          break;
        case 2:
          Hands.RotateHands(speed,1);
          break;
        case 3:
          Hands.RotateHands(speed,0);
          break;
      }
    }

    for(int i=0;i< 6;i++)
    {
      lastDMX[i]= DMX[i];
    }


    
  }else if((millis()-DMXtime) > 60000)
  {//Data lost
    digitalWrite(DMXled, 0);
    Hands.MoveToTime(6, 30, 0, 100, 1);
    while(Hands.Tick()){}
    Hands.setSleep(true);
    sleepState = true;
    for(int i=0;i< 6;i++)
    {
      lastDMX[i]= 0;
    }
  }


  if(!Hands.Tick()){
    speed = map(DMX[5],0,255,0,500);
      switch(DMX[4]){
        case 0:
          Hands.stop();
          break;
        case 1:
          Hands.RotateRealTime(1);
          break;
        case 2:
          Hands.RotateHands(speed,1);
          break;
        case 3:
          Hands.RotateHands(speed,0);
          break;
      }
  }




  Ethernet.maintain();







//   Hands.MoveToTime(12 , 30, 0, 500, 0);
//
//  while(Hands.Tick()){}
//   delay(500);
//   Hands.RotateRealTime(1);
////   Hands.MoveToTime(12, 0, 0, 200, 1);
//   while(Hands.Tick()){}
////   delay(500);
////   Hands.MoveToTime(1, 30);
//////   delay(5000);
//////   Hands.MoveToTime(7, 13);
//////   delay(5000);
////   Hands.MoveToTime(12, 59);
////  delay(10000);
////  Serial.print("loop ");
////  Serial.println(loopCount);
////  loopCount++;
////Serial.println(digitalRead(16));

//delay(5000);
}
