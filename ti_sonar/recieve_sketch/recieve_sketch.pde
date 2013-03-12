unsigned long in;
void setup(){
  Serial.begin(9600);
  Serial3.begin(9600);
}
void loop(){
  byte dir =0;
  byte readin[4];
  byte buf;
  int avail;
//  if(avail%4!= 0)
//    while((in = Serial3.read())==-1){}

  while((avail=Serial3.available())/4>0){
    for(int i = 0;i<4;i++){
        readin[i] = Serial3.read();
    }
    in = *((unsigned long*)readin);
    
    dir = (in&0xff000000)>>24;
    in &= 0x00ffffff;
    
    byte dirdist;
    dirdist = dir & 0x1f;
    dir = (dir>>5)&0x7;
  
   
    Serial.println();
    Serial.print(avail);
    Serial.print(" - ");
    Serial.print(dir,DEC);
    Serial.print("@");
    Serial.print(dirdist,DEC);
    Serial.print(" ");
    Serial.println(in,DEC);
  }
  delay(10); // simulate operating frequency

}
