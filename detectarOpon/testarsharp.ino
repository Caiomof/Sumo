/*
 * getDstance
 *
 * Example of using SharpIR library to calculate the distance beetween the sensor and an obstacle
 *
 * Created by Giuseppe Masino, 15 June 2016
 * Author URL http://www.facebook.com/dev.hackerinside
 * GitHub URL http://github.com/HackerInside0/Arduino_SharpIR
 *
 * -----------------------------------------------------------------------------------
 *
 * Things that you need:
 * - Arduino
 * - A Sharp IR Sensor
 *
 *
 * The circuit:
 * - Arduino 5V -> Sensor's pin 1 (Vcc)
 * - Arduino GND -> Sensor's pin 2 (GND)
 * - Arduino pin A0 -> Sensor's pin 3 (Output)
 *
 */

//import the library in the sketch
#include <SharpIR.h>

//Create a new instance of the library
//Call the sensor "sensor"
//The model of the sensor is "GP2YA41SK0F"
//The sensor output pin is attached to the pin A0
SharpIR sensor0(GP2YA41SK0F, A0); // DIREITA
SharpIR sensor1(GP2YA41SK0F, A2); // MEIO
SharpIR sensor2(GP2YA41SK0F, A1); // ESQUERDA

int distance[3] = {0};

void setup()
{
    Serial.begin(9600); //Enable the serial comunication
}

void loop()
{
    int i = 0;

    distance[0] = sensor0.getDistance(); //Calculate the distance in centimeters and store the value in a variable
    distance[1] = sensor1.getDistance(); //Calculate the distance in centimeters and store the value in a variable
    distance[2] = sensor2.getDistance(); //Calculate the distance in centimeters and store the value in a variable

    for(i = 0; i < 3; i++)
    {
        Serial.print("Distancia "); Serial.print(i); Serial.print(" : ");
        Serial.println(distance[i]); //Print the value to the serial monitor
        delay(250);
    }
    Serial.println(); 
}
