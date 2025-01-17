#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial
#define Baudrate 9600

#include <SoftwareSerial.h>

// Definieer de pinnen voor SoftwareSerial
SoftwareSerial bluetooth(2, 3); // RX, TX (Pin 10 voor RX en 11 voor TX)



#define IN1 10
#define IN2 9
#define IN3 6   
#define IN4 5

SerialCommand sCmd(SerialPort);
bool debug;
unsigned long previous, calculationTime;
bool run;
float iTerm = 0;
float lastErr; 
const int sensor[] = { A5, A4, A3, A2, A1, A0};  
const int Drukknop = 13;
const int led = 12; 
int normalised[6];
float debugPosition;
float output;
const byte LED=6;
const byte knop=3;


struct param_t
{
  unsigned long cycleTime;
  /* andere parameters die in het eeprom geheugen moeten opgeslagen worden voeg je hier toe ... */
  int black[6];
  int white[6];
  int power; 
  float diff;
  float kp;  
  float ki; 
  float kd;
} params;

void setup()
{
SerialPort.begin(Baudrate);
bluetooth.begin(Baudrate); // set serial baudrate at 115200

  sCmd.addCommand("set", onSet);
  sCmd.addCommand("debug", onDebug);
  sCmd.addCommand("calibrate", onCalibrate); 
  sCmd.addCommand("run", onRun);
  sCmd.addCommand("stop", onStop); 
  sCmd.setDefaultHandler(onUnknownCommand);
  EEPROM_readAnything(0, params);

  pinMode(LED, OUTPUT);
  pinMode(knop,INPUT);
  attachInterrupt(digitalPinToInterrupt(knop), toggle, RISING);

  pinMode(12, OUTPUT); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  SerialPort.println("ready");
}


void loop()
{
/*
  if (run)
  {
    digitalWrite(LED,HIGH);
    delay(500);
    digitalWrite(LED, LOW);
  delay(500);
  }
  else digitalWrite(LED,LOW);
 */
  


  sCmd.readSerial();

  if (Serial.available() > 0) 
  {
    String teststr = Serial.readString();  //read until timeout
    teststr.trim();                        // remove any \r \n whitespace at the end of the String
    bluetooth.println(teststr);
  }


  if (bluetooth.available())
  {
  Serial.write(bluetooth.read());
  }
 
  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;

    /* code die cyclisch moet uitgevoerd worden programmeer je hier ... */
      for (int i = 0; i < 6; i++)
    {
      //SerialPort.print(analogRead(sensor[i]));
      //SerialPort.print(" ");

      normalised[i] = map(analogRead(sensor[i]), params.black[i], params.white[i], 0, 1000); 
    }
    //SerialPort.println(" ");
    float position = 0; // positie bepalen sensoren 
    int index = 0; // zwartste sensor bepalen 
    for(int i = 1; i < 8; i++) if (normalised[i] < normalised[index]) index = i; // nieuwe zwartste sensor zoeken 
    //SerialPort.println(normalised[index]);

    if (normalised[index] > 1050 ) run = false; 

    if (index == 0) position = -30; 
    else if (index == 7) position = 30;
    else
    {
      int sensor_nul = normalised[index];
      int sensor_min_een = normalised[index-1];
      int sensor_plus_een = normalised[index+1];

      float b = sensor_plus_een - sensor_min_een;  // b berekenen 
      b = b / 2; 

      float a = sensor_plus_een - b - sensor_nul; 

      position = -b / (2 * a); 
      position += index;
      position -= 3.5;

      position *= 9.525;      
    }
    debugPosition = position; 
    
   
    float error = -position;

    
    output = error * params.kp;

    iTerm += params.ki * error;
    iTerm = constrain(iTerm, -510, 510);
    output += iTerm;


    output += params.kd * (error - lastErr);
    lastErr = error; 
    
   
    output = constrain(output, -510, 510); 
    
    int powerLeft = 0;
    int powerRight = 0;
    
    if (run) if (output >= 0)
    {
      powerLeft = constrain(params.power + params.diff * output, -255, 255); 
      powerRight = constrain(powerLeft - output, -255, 255); 
      powerLeft = powerRight + output; 
    }
    else
    {
      powerRight = constrain(params.power - params.diff * output, -255, 255); 
      powerLeft = constrain(powerRight + output, -255, 255); 
      powerRight = powerLeft - output;
    }

    SerialPort.print("powerLeft");
    SerialPort.println(powerLeft);
    
    SerialPort.print("powerRight"); 
    SerialPort.println(powerRight);
    
    //H-brug waardes wegschrijven 
    
    analogWrite(IN1, powerLeft > 0 ? powerLeft : 0);
    analogWrite(IN2, powerLeft < 0 ? -powerLeft : 0);
    analogWrite(IN3, powerRight > 0 ? powerRight : 0);
    analogWrite(IN4, powerRight < 0 ? -powerRight : 0);
   /*
    //kruispunt
    bool kruispunt = LOW;

    for(int i= 0; i< 6; i++)
    {
      if(normalised[i]< 500)
      {
        kruispunt= HIGH;
      }
      else
      {
        kuispunt = LOW;
      }
      if (kruispunt)
      {
        powerRight=params.power;
        powerLeft= params.power;
      }
    }
    */
    if (powerRight >= 0)
      {
        powerRight= powerRight*1.5;
        digitalWrite(IN4, powerRight);//rechts
        analogWrite(IN2, powerRight);

      }
    else if (powerRight < 0)
      {
        digitalWrite(IN4, HIGH);
        analogWrite(IN2, LOW);
       
      }
     if (powerLeft >= 0) 
       {
        powerLeft=1.5*powerLeft;
          digitalWrite(IN3, powerLeft);
          analogWrite(IN1, powerLeft);

       }
     else if (powerLeft < 0) 
       {
          digitalWrite(IN3, HIGH);
          analogWrite(IN1, LOW);
          
       }






  } 






  

  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;
}

void onUnknownCommand(char *command)
{
  SerialPort.print("unknown command: \"");
  SerialPort.print(command);
  SerialPort.println("\"");
}

void onSet()
{
  char* param = sCmd.next();
  char* value = sCmd.next();  
  
if (strcmp(param, "cycle") == 0)
  {
    long newCycleTime = atol(value);
    float ratio = ((float) newCycleTime) / ((float) params.cycleTime);

    params.ki *= ratio;
    params.kd /= ratio; 

    params.cycleTime = newCycleTime;
  }
  else if (strcmp(param, "ki") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.ki = atof(value) * cycleTimeInSec;
  }
  else if (strcmp(param, "kd") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.kd = atof(value) / cycleTimeInSec;
  }

  else if (strcmp(param, "cycle") == 0) params.cycleTime = atol(value); // if 
  else if (strcmp(param, "power") == 0) params.power = atol(value); 
  else if (strcmp(param, "diff") == 0) params.diff = atof(value);
  else if (strcmp(param, "kp") == 0) params.kp = atof(value);

  
  EEPROM_writeAnything(0, params);
}

void onDebug()
{
  SerialPort.print("cycle time: ");
  SerialPort.println(params.cycleTime);
  
  /* parameters weergeven met behulp van het debug commando doe je hier ... */
  SerialPort.print("black: ");
  for (int i = 0; i < 6; i++)
  {
    SerialPort.print(params.black[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("white: ");
  for (int i = 0; i < 6; i++)
  {
    SerialPort.print(params.white[i]);
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("normalised: ");
  for (int i = 0; i < 6; i++)
  {
    SerialPort.print(normalised[i]);
    SerialPort.print(" "); 
  }
  SerialPort.println(" ");

  SerialPort.print("output: ");
  SerialPort.println(output); 

  SerialPort.print("power: ");
  SerialPort.println(params.power);
  SerialPort.print("diff: ");
  SerialPort.println(params.diff);
  SerialPort.print("kp: ");
  SerialPort.println(params.kp);

  float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
  float ki = params.ki / cycleTimeInSec;
  SerialPort.print("ki: ");
  SerialPort.println(ki);

  float kd = params.kd * cycleTimeInSec;
  SerialPort.print("kd: ");
  SerialPort.println(kd);
  
  SerialPort.print("calculation time: ");
  SerialPort.println(calculationTime);
  calculationTime = 0;
}
void onCalibrate()
{
  char* param = sCmd.next();

  if (strcmp(param, "black") == 0)
  {
    SerialPort.print("start calibrating black... ");
    for (int i = 0; i < 6; i++) params.black[i] = analogRead(sensor[i]);   
    SerialPort.println("done");
    
  }
  else if (strcmp(param, "white") == 0)
  {
    SerialPort.print("start calibrating white... ");
    for (int i = 0; i < 6; i++) params.white[i] = analogRead(sensor[i]);   
    SerialPort.println("done");
  }
  EEPROM_writeAnything(0, params);
}
void onRun()
{
  run =true;
  digitalWrite(led, HIGH); 
}
void onStop()
{
  run =false; 
  digitalWrite(led, LOW);
}
void toggle()
{
run= !run;
 }