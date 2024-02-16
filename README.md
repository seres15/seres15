#include <Servo.h>                                //Importo linreria de control del servo

Servo servoLook;                                  //Creo un objecto para controlar el servo

byte trig = 10;                                    //Asigno pines al sensor ultrasonico
byte echo = 9;
byte maxDist = 150;                               //Distancia maxima de sensado (Objects further than this distance are ignored)
byte stopDist = 50;                               //Minimum distance from an object to stop in cm
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a return signal

void setup() 
{
  pinMode(8,OUTPUT);      //motor izquierdo delantero
  pinMode(7,OUTPUT);      //motor izquierdo trasero
  pinMode(6,OUTPUT);      //motor derecho delantero
  pinMode(5,OUTPUT);      //motor derecho trasero
  servoLook.attach(3);    //Asigno pin al servo
  pinMode(trig,OUTPUT);   
  pinMode(echo,INPUT);
}

void loop() 
{
  servoLook.write(90);                            //Set the servo to look straight ahead
  delay(750);
  int distance = getDistance();                   //Check that there are no objects ahead
  if(distance >= stopDist)                        //If there are no objects within the stopping distance, move forward
  {
    moveForward();
  }
  while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    delay(250);
  }
  stopMove();                                     //Stop the motors
  int turnDir = checkDirection();                 //Check the left and right object distances and get the turning instruction
  Serial.print(turnDir);
  switch (turnDir)                                //Turn left, turn around or turn right depending on the instruction
  {
    case 0:                                       //Turn left
      turnLeft (700);
      break;
    case 1:                                       //Turn around
      turnLeft (1500);
      break;
    case 2:                                       //Turn right
      turnRight (700);
      break;
  }
}

void moveForward()                                //Set all motors to run forward
{
  digitalWrite(8,HIGH);
  digitalWrite(6,HIGH);
}

void stopMove()                                   //Set all motors to stop
{
  digitalWrite(8,LOW);
  digitalWrite(7,LOW);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
}

void turnLeft(int duration)                                 //Set motors to turn left for the specified duration then stop
{
  digitalWrite(8,HIGH);
  delay(duration);
  digitalWrite(8,LOW);
  
}

void turnRight(int duration)                                //Set motors to turn right for the specified duration then stop
{
  digitalWrite(6,HIGH);
  delay(duration);
  digitalWrite(6,LOW);
}

int getDistance()                                   //Measure the distance to an object
{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //Measure the time for the pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object distance based on the pulse time
  return distance;
}

int checkDirection()                                            //Check the left and right directions and decide which way to turn
{
  int distances [2] = {0,0};                                    //Left and right distances
  int turnDir = 1;                                              //Direction to turn, 0 left, 1 reverse, 2 right
  servoLook.write(180);                                         //Turn servo to look left
  delay(500);
  distances [0] = getDistance();                                //Get the left object distance
  servoLook.write(0);                                           //Turn servo to look right
  delay(1000);
  distances [1] = getDistance();                                //Get the right object distance
  if (distances[0]>=200 && distances[1]>=200)                   //If both directions are clear, turn left
    turnDir = 0;
  else if (distances[0]<=stopDist && distances[1]<=stopDist)    //If both directions are blocked, turn around
    turnDir = 1;
  else if (distances[0]>=distances[1])                          //If left has more space, turn left
    turnDir = 0;
  else if (distances[0]<distances[1])                           //If right has more space, turn right
    turnDir = 2;
  return turnDir;
}
