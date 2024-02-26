// /*In order to predict when the battery needs charging, first some simple
// calculations has to be done.

//General information about LiPo batteries, says that battery safety limits are:
// » 2.75~3.0V is the voltage when the battery is empty, and
// » 3.95~4.2V is the voltage when the battery is fully charged.

#include "batterymonitor.h"
#include "Arduino.h"
#include <string>

//Analog pin on ESP32 has a resolution of 12-bit
#define sensivity (3.3 / 4096.0) 

// Map raw readings 
float mapf(float x, float in_min, float in_max, float out_min,
float out_max) {
float a = x - in_min;
float b = out_max - out_min;
float c = in_max - in_min;
return a * b / c + out_min;
}

//Function to display battery voltage 
float batterymonitor () {
   // Raw analog reading 
    float sensorValue = analogRead(A0);
    String battery_state [4] = {"Nominal Voltage", "Low Voltage", "Completely Discharged", "No Battery"};

    //Conversion to voltage
    float A0_Voltage = sensorValue * sensivity;
    Serial.print("A0 Voltage: ");
    Serial.print(A0_Voltage);
    Serial.println("V");

    if (A0_Voltage >= 2.67) {
        Serial.println(battery_state[0]);
        
    }
    else if (A0_Voltage <= 1.90 && A0_Voltage >= 1.78){
        Serial.println(battery_state[1]);
        
    }

    else if(A0_Voltage <= 1.78 && A0_Voltage > 0.00){
        Serial.println(battery_state[2]);

    }

    else if(A0_Voltage <= 0.00){
        Serial.println(battery_state[3]);
    }

    return A0_Voltage;
    
}
