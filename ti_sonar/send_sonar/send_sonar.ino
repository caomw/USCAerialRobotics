#include <math.h>

//available pins 2,5,6,7,8,9,10 - 11,12,13,(14,15),(18,19)
#define trig1 2
#define echo1 5
#define trig2 6
#define echo2 7
#define trig3 8
#define echo3 9
#define trig4 11
#define echo4 12
#define ping 10
//pin 3 = rxd pin 4 = txd for serial com.

#define INTERDELAY 4
#define PULSEDUR 10000

const char trig[4] = {trig1,trig2,trig3,trig4};
const char echo[4] = {echo1,echo2,echo3,echo4};

byte passlmap(long dist);
byte passemap(long dist);

void setup(){
  for(int i = 0; i<4;i++){
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  Serial.begin(9600);

}

void loop(){
  unsigned long duration[4];
  unsigned long cm[4];
  unsigned long hmm, hdur;
  byte sendout[4];
  
// pull in height
  pinMode(ping, OUTPUT);
  digitalWrite(ping, LOW);
  delayMicroseconds(2);
  digitalWrite(ping,HIGH);
  delayMicroseconds(5);
  digitalWrite(ping,LOW);
  pinMode(ping,INPUT);
  hdur = pulseIn(ping, HIGH, PULSEDUR);
  hmm = hdur*10 /29 /2;
  delay(INTERDELAY);

  for(int i = 0; i<4;i++){
    digitalWrite(trig[i],HIGH);
    delayMicroseconds(1000);
    digitalWrite(trig[i],LOW);
    duration[i] = pulseIn(echo[i],HIGH,PULSEDUR);
    delay(INTERDELAY);
  }
  
  int imin = -1;
  long vmin = 50000;
  long nmin;
  //calculate all of the distances and 
  for(int i = 0;i<4;i++){
    if(duration[i] == 0)
      duration[i] = 10000000;
      
    nmin = (cm[i] = duration[i] / 29 /2);
    if(nmin<vmin){
      imin = i;
      vmin = nmin;
    }

  }
  byte dirbyte = 0x00;
// pack the output character for serial communication
  
  dirbyte = imin;
  
  sendout[0] = hmm & 0x000000ff;
  sendout[1] = (hmm & 0x0000ff00)>>8;
  sendout[2] = (hmm & 0x00ff0000)>>16;
  sendout[3] = dirbyte;
  
  Serial.write(sendout,4);
  
}


