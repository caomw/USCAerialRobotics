#include <Wire.h>
#include <APM_RC.h>
#include <APM_ADC.h>
#include <APM_Compass.h>
#include <APM_BMP085.h>
//#include <GPS_NMEA.h>

#include "ART_Quadrotor_old.h"

#define USE_SERIAL
#define trigPin 14
#define echoPin 15

void setup() {
#ifdef USE_SERIAL
  Serial.begin(BAUDRATE);
#endif

  Wire.begin();

  pinMode(LEDYELLOW,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(SWITCH1,INPUT);
  pinMode(SWITCH2,INPUT);  

  //for sonar pwm
  pinMode(15, INPUT);  

  APM_RC.Init();
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);  

  APM_ADC.Init();
  //APM_BMP085.Init();
  //APM_Compass.Init();
  //GPS.Init();
  delay(100);  

  // Calibrate Gyros
  digitalWrite(LEDRED,HIGH);

  float aux_float[3];

  ADC_Offset[0] = gyroOffset[0];
  ADC_Offset[1] = gyroOffset[1];
  ADC_Offset[2] = gyroOffset[2];  
  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  readADC();
  delay(20);

  long averageGyro[3] = {
    0,0,0  
  };

  for ( int i = 0 ; i < 100 ; i++ ) {
    readADC();
    for ( int j = 0 ; j < 3 ; j++ ) {
      averageGyro[j] += ADC_Ch[j];
    }
    delay(30);
  }
  for ( int i = 0 ; i < 3 ; i++ ) {
    ADC_Offset[i] = averageGyro[i]/100;    
  }

  digitalWrite(LEDRED,LOW);

  calibrateLevel();

  /* Init pressure altitude
  for ( unsigned int i = 0 ; i < 1000 ; i++ ) {
    readPressureAltitude();
  }
  
  groundPressureAltitude = pressureAltitude;
*/
  timer1 = millis();
  compassReadTimer = millis();  
  telemetryTimer = millis();
  otherTimer = millis();
}


// PID Tuning Packet
union PIDPacket
{
  struct
  {
    float roll[3];
    float pitch[3];
    float yaw[3];
    float altitude[3];
  };
  char data[48];
};


void loop() {

  //Sonar Sensor Data
  if ( millis() - sonarTimer > 100 ) {
    loopDt_slow = (millis() - sonarTimer)/1000.0;
    sonarTimer = millis();
    
    
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    previousAltitude = sonarAltitude;
    
    int duration;
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH,10000);
    sonarReading = (duration/2) / 29.1;
    if( abs(sonarReading - previousReading) > 20)
    {
      previousReading = sonarReading;
      sonarReading = sonarAltitude;
    }
    else 
    {
      previousReading = sonarReading;
    }
    
    if( sonarReading < 2)
    {
      sonarReading = 150;
    }
    
    
    
    float lambda = 0.25;
    sonarAltitude = sonarAltitude*(1-lambda) + sonarReading*lambda;
    
    actualAltitude = sonarAltitude;
    usedAltitude = sonarAltitude;

    
    Serial.println(sonarAltitude);
    
    delay(10);
    }


  if ( millis() - timer1 > 10 ) { // timer at 100 Hz
 
    loopDt = (millis()-timer1)/1000.0;
    timer1 = millis();

    timeStep++;
    getMeasurements(); 

    // Read RC receiver
    for ( int i = 0 ; i < 4 ; i++ ) {
      RCInput[i] = radioFilter(APM_RC.InputCh(i),RCInput[i]);
    }

    for ( int i = 4 ; i < 8 ; i++ ) {
      RCInput[i] = APM_RC.InputCh(i);
    }

    pilotRollOld = pilotRoll;
    pilotRoll = 0.105*(RCInput[0]-MIDCHANNEL);
    pilotPitchOld = pilotPitch;
    pilotPitch = -0.105*(RCInput[1]-MIDCHANNEL);
    pilotThrottle = RCInput[2];
    //Serial.println(pilotThrottle);
    pilotYawOld = pilotYaw;
    pilotYaw = (RCInput[3]-MIDCHANNEL);
   
    if (RCInput[4] < 1550) //Altitude hold will be on
    {
      if (altHold == false) 
      {
        //holdingAltitude = sonarAltitude;
        //desiredAltitude = 60.0;
        holdingThrottle = pilotThrottle;
        
        altHold = true;
      }
      throttle = holdingThrottle;
      //Serial.println(throttle);
      //desiredAltitude = 60.0;
      desiredAltitude = (pilotThrottle - 1100)/7;
   }
   else
   {
     altHold = false;
     throttle = pilotThrottle;
     desiredAltitude = sonarAltitude;
   }


    PIDControl();
    
    
    if ( pilotThrottle < 1200 ) {
    
     
      if ( RCInput[3] > 1700 && RCInput[5] > 1550) {
        motorsArmed = 1;
      }
      if ( RCInput[3] < 1200 ) {
        motorsArmed = 0;
        isLanding = false;
      }
    }
   if (motorsArmed == 0) {
      altitudeI = 0; 
      yawI = 0;
      rollI = 0;
      pitchI = 0;
   }
   if (altHold == false)
     altitudeI = 0;

 // DO NOT DELETE: THIS IS FOR COMPUTER CONTROLLED ALTITUDE HOLD
  // Computer controlled mode
       digitalWrite(LEDYELLOW,HIGH); 
       //Serial.println("ALT HOLD");       
  
    /*   
    if ( isLanding == true ) {
       desiredAltitude = constrain(landingAltitude - ((millis() - landingTime)/75),0,200);
         if (sonarAltitude < 18) {//When vehicle is close to the ground, kill motors
           isLanding = false;
           holdingAltitude = false;
           motorsArmed = 0;
           }        
         }
         */
      
      
    isManualControl = true;
  
    digitalWrite(LEDYELLOW,LOW);  
       
   
     
     
    throttle = pilotThrottle;
     
        //controlAltitude = 0;
    digitalWrite(LEDYELLOW,HIGH); 
     
   

    digitalWrite(LEDYELLOW,LOW);
  
    if ( motorsArmed == 1 ) {
      digitalWrite(LEDRED,HIGH);    
      if (altHold == false)
        controlAltitude = 0;
        
      motor[0] = constrain(throttle+controlRoll-controlPitch+controlYaw+(controlAltitude)/*+controlXpose+controlYpose-controlThetapose*/,1100,2000);
      motor[1] = constrain(throttle+controlRoll+controlPitch-controlYaw+(controlAltitude)/*+controlXpose-controlYpose+controlThetapose*/,1100,2000);
      motor[2] = constrain(throttle-controlRoll+controlPitch+controlYaw+(controlAltitude)/*-controlXpose-controlYpose-controlThetapose*/,1100,2000);
      motor[3] = constrain(throttle-controlRoll-controlPitch-controlYaw+(controlAltitude)/*-controlXpose+controlYpose+controlThetapose*/,1100,2000);
      
      
      APM_RC.OutputCh(0,motor[0]);
      APM_RC.OutputCh(1,motor[1]);
      APM_RC.OutputCh(2,motor[2]);
      APM_RC.OutputCh(3,motor[3]);
    } 
    else {
      digitalWrite(LEDRED,LOW);      
      APM_RC.OutputCh(0,1100);
      APM_RC.OutputCh(1,1100);
      APM_RC.OutputCh(2,1100);
      APM_RC.OutputCh(3,1100);
    }
  }
 
}

void PIDControl() {
  rollError = constrain(pilotRoll - ToDeg(roll),-60,60);

  rollI += rollError*loopDt;
  rollI = constrain(rollI,-pitchRollMax,pitchRollMax);

  rollD = ToDeg(-Omega[0]);

  controlRoll = Kproll*rollError + Kiroll*rollI + Kdroll*rollD;

  ///////////////////////////////////////

  pitchError = constrain(pilotPitch - ToDeg(pitch),-60,60);

  pitchI += pitchError*loopDt;
  pitchI = constrain(pitchI,-pitchRollMax,pitchRollMax);

  pitchD = ToDeg(-Omega[1]);

  controlPitch= Kppitch*pitchError + Kipitch*pitchI + Kdpitch*pitchD;

  /////////////////////////////////////////////

  yawError = constrain(pilotYaw - .5*readADCCorrected(2),-80,80);

  yawI += yawError*loopDt;
  yawI = constrain(yawI,-20,20);

  //yawD = readADCCorrected(2);

  controlYaw= Kpyaw*yawError + Kiyaw*yawI + Kdyaw*yawD;

  /////////////////////////////////////////////

  altitudeError = desiredAltitude-sonarAltitude;

  altitudeI += altitudeError*loopDt;
  altitudeI = constrain(altitudeI,-400,400);

  altitudeD = -(sonarAltitude - previousAltitude)/loopDt_slow;

  controlAltitude = Kpaltitude*altitudeError + Kialtitude*altitudeI + Kdaltitude*altitudeD;  

}

// Maximum slope filter for radio inputs... (limit max differences between readings)
int radioFilter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
  {
    if (diff_ch_old<-40)
      return(ch_old-40);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old>40)    
      return(ch_old+40);
  }
  return(ch);
}

void getMeasurements() {
  readADC();
  if (MAGNETOMETER == 1) {
    if (millis() - compassReadTimer >= 100)  // Read compass data at 10Hz...
    {
      compassReadTimer = millis();
      APM_Compass.Read();     // Read magnetometer
      APM_Compass.Calculate(roll,pitch);  // Calculate heading
    }
  }
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();

  //  sonarAltitude = sonarAltitude*.9 + constrain(analogRead(A6),15,500)*.1;
}

void calibrateLevel() {
  digitalWrite(LEDRED,HIGH);

  // Calibrate accelOffset
  int average_x_offset = 0;
  int average_y_offset = 0;

  for ( int i = 0 ; i < 10 ; i++ ) {
    readADC();
    average_x_offset += ADC_Ch[3];
    average_y_offset += ADC_Ch[4];
    delay(20);
  }

  accelOffset[0] = average_x_offset/10;
  accelOffset[1] = average_y_offset/10;

  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  digitalWrite(LEDRED,LOW);
}

void readPressureAltitude() {
  // Barometer read
  APM_BMP085.Read();
  float tempPressureAltitude;
  tempPressureAltitude = float(APM_BMP085.Press)/(101325.0); //101325.0;
  tempPressureAltitude = pow(tempPressureAltitude,0.190295);
  if ( pressureAltitude == 0 ) {
    pressureAltitude = (1.0 - tempPressureAltitude) * 4433000 / 2.54; // in
  } 
  else {
    pressureAltitude = pressureAltitude * 0.94 + ((1.0 - tempPressureAltitude) * 4433000 / 2.54)*0.06; // in
  }
  //Serial.println(pressureAltitude - groundPressureAltitude);
}  
