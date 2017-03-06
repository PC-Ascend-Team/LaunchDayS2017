/*
Phoenix College Spring 2017 Launch Code





HEADER DEFINITION:
example: "roll,pitch,heading,accelx(g),accely(g),accelz(g),magx(gaus),magy(gaus),magz(gaus),gyrox(dps),gyroy(dps),gyroz(dps),pres(hPa),imuAlt(m),imuTemp(F),Temp0(F),Temp1(F),CO2,Geiger(cpm),GPS,millis(ms),APRS"

roll        =IMU rotational roll in degrees
pitch       =IMU rotational pitch in degrees
heading     =IMU rotational yaw in degrees
accelx(g)      =acceleration in the x direction in G forces where 9.81m/s/s is 1G
accely(g)      =acceleration in the y direction in G forces where 9.81m/s/s is 1G
accelz(g)      =acceleration in the z direction in G forces where 9.81m/s/s is 1G
magx(gaus)        =magnemometer axis x, outputting gauss units
magy(gaus)        =magnemometer axis y, outputting gauss units
magz(gaus)        =magnemometer axis z, outputting gauss units
gyrox(dps)       =Gyro rotation in the x axis direction in degrees per second (dps) units
gyroy(dps)       =Gyro rotation in the y axis direction in degrees per second (dps) units
gyroz(dps)       =Gyro rotation in the z axis direction in degrees per second (dps) units
pres(hPa)        =Pressure from the IMU board outputting hector pascals (hPa)
clacAlt(m)  =altitude calculated by the 10DOF IMU using pressure, temperature, and local sea level. Outputs meters
imuTemp(C)  =temperature data from the BMP180 on the 10DOF IMU, outputting in celcius.
Temp0(F)    =TMP36 sensor data in fahrenheit for the TMP36 on the inside of the payload
Temp1(F)    =TMP36 sensor data in fahrenheit for the TMP36 on the outside of the payload
CO2         =output reserved for CO2 sensor. will be updated when implemented
Geiger(cpm) =general geiger counter in counts per minute. Will be updated when we implement more geiger counters
GPS         =gps output, will be updated with more placements later. Currently planning to capture GPGGA type NMEA GPS sentences.
millis(ms)  =the duration that the current program has been running in milliseconds. If we have no GPS fix, we have no sense of time. This uses the arduino local clock to tell how much time has passed, and as indication of system failures
APRS        =placeholder for data from APRS. (this should just be a transmitter, but we should also be able to get location from it as well... maybe...)

*/



////////////////////////////////////////////////////////////////////////////////
//               Start of including headers
////////////////////////////////////////////////////////////////////////////////


#include <Wire.h>               //I2C class
#include <Adafruit_Sensor.h>    //general sensor class
#include <Adafruit_LSM303_U.h> //IMU (accel, mag)
#include <Adafruit_BMP085_U.h> //IMU (BMP)
#include <Adafruit_L3GD20_U.h> //IMU (Gryo)
#include <Adafruit_Simple_AHRS.h> //for calculating the roll, pitch and yaw


////////////////////////////////////////////////////////////////////////////////
//                End of including headers
////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////
//   Declare globals like pin numbers or constants/volatile variables here.
////////////////////////////////////////////////////////////////////////////////

char delimiter = ',';   //used for seperating sensor values in the logging file

int temp0Pin = A0;             //I'm writing A0 because you guys seem to use that. I use just 0. Both are valid.
int temp1Pin = A1;

int geigerGeneralPin = 2;
// int geigerAlphaPin = ;   //when these are bought and tested, add these pins
// int geigerBetaPin = ;
// int geigerGammaPin = ;


//IMU globals
// Create sensor instances.
Adafruit_LSM303_Accel_Unified A (30301); //accelerometer
Adafruit_LSM303_Mag_Unified   M (30302); //Magnetometer
Adafruit_BMP085_Unified       B (18001); //Pressure/temp
Adafruit_L3GD20_Unified       G  (20);   //Gyro
// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&A, &M);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
//end of imu gloabls


////////////////////////////////////////////////////////////////////////////////
//                          End of global variables
////////////////////////////////////////////////////////////////////////////////









////////////////////////////////////////////////////////////////////////////////
//                 Start of interrupt function
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//                 End of interrupt function
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
//                          Start of setup
////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);

    //the F() function stores the entire string literal only in flash memory, which saves a bit of program space. Better to add it now, then to have issues later and take a long time to diagnose and fix. This is preventative
    Serial.println( F("roll,pitch,heading,accelx(g),accely(g),accelz(g),magx(gaus),magy(gaus),magz(gaus),gyrox(dps),gyroy(dps),gyroz(dps),pres(hPa),imuAlt(m),imuTemp(F),Temp0(F),Temp1(F),CO2,Geiger(cpm),GPS,millis(ms),APRS") );

    //IMU initialization
    A.begin();//accel
    M.begin();//magnemometer
    B.begin();//barometer
    G.begin();//gyro
    //end of IMU initialization

}//end of setup
////////////////////////////////////////////////////////////////////////////////
//                          End of setup
////////////////////////////////////////////////////////////////////////////////














////////////////////////////////////////////////////////////////////////////////
//                          Start of loop
////////////////////////////////////////////////////////////////////////////////
void loop() {


    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of IMU Euler/Raw
    ////////////////////////////////////////////////////////////////////////////////

    sensors_vec_t   orientation;
    // Uses the AHRS function to get the current orientation (roll, pitch, heading/yaw) and prints data
    if (ahrs.getOrientation(&orientation))
    {
        Serial.print(orientation.roll);
        Serial.print(delimiter);
        Serial.print(orientation.pitch);
        Serial.print(delimiter);
        Serial.print(orientation.heading);
        Serial.print(delimiter);
    }

    //raw sensor datar
    sensors_event_t A_event;
    sensors_event_t M_event;
    sensors_event_t G_event;
    sensors_event_t B_event;

    //gets a sensor event from each sensor
    A.getEvent(&A_event);
    M.getEvent(&M_event);
    G.getEvent(&G_event);

    //Display Raw Sensor Information: Accel, Mag, Gyro
    Serial.print(A_event.acceleration.x); Serial.print(delimiter);
    Serial.print(A_event.acceleration.y); Serial.print(delimiter);
    Serial.print(A_event.acceleration.z); Serial.print(delimiter);
    Serial.print(M_event.magnetic.x);     Serial.print(delimiter);
    Serial.print(M_event.magnetic.y);     Serial.print(delimiter);
    Serial.print(M_event.magnetic.z);     Serial.print(delimiter);
    Serial.print(G_event.gyro.x);         Serial.print(delimiter);
    Serial.print(G_event.gyro.y);         Serial.print(delimiter);
    Serial.print(G_event.gyro.z);         Serial.print(delimiter);

    sensors_event_t bmp_event;
    B.getEvent(&B_event);
    Serial.print(B_event.pressure);     Serial.print(delimiter); //prints pressure
    if (B_event.pressure)
    {
        // Get temperature
        float temperature;
        B.getTemperature(&temperature);
        //Convert atmospheric pressure, SLP and temp to altitude (get altitude)
        Serial.print(B.pressureToAltitude(seaLevelPressure, B_event.pressure, temperature)); //prints altitude
        Serial.println(temperature);
        Serial.print(delimiter);

    }
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of IMU Euler/Raw
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of Temp0(F)
    ////////////////////////////////////////////////////////////////////////////////
    float temp0 = getTempF(temp0Pin);
    Serial.print(temp0);
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of Temp0(F)
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of Temp1(F)
    ////////////////////////////////////////////////////////////////////////////////
    float temp1 = getTempF(temp1Pin);
    Serial.print(temp1);
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of Temp1(F)
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of CO2
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("CO2");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of CO2
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of Geiger(cpm)
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("Geiger(cpm)");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of Geiger(cpm)
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of GPS
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("GPS");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of GPS
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of millis(ms)
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("millis(ms)");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of millis(ms)
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of APRS
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("APRS");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of APRS
    ////////////////////////////////////////////////////////////////////////////////






}//end of loop
////////////////////////////////////////////////////////////////////////////////
//                          End of loop
////////////////////////////////////////////////////////////////////////////////
















////////////////////////////////////////////////////////////////////////////////
//                     Start of function definitions
////////////////////////////////////////////////////////////////////////////////

//takes an integer pinNumber and returns the voltage from that analog pin
float getVoltage(int pinNumber){
    float voltage = analogRead (sensorPin);
    return voltage*(5.0/1023.0);
}

//takes a voltage and outputs a temperature in celcius assuming its from a TMP36
float voltageToTempC(float voltage ){
    float voltageToCel = (voltage - 0.5) * 100;
    return voltageToCel;
}


//takes a voltage and outputs a temperature in fahrenheight assuming its from a TMP36
float tempCToTempF(float tempC){
    float celToFaren = (tempC * 9.0 / 5.0) + 32.0;
    return celToFaren;
}


//uses the tempCToTempF and voltageToTempC to get a direct voltage to fahrenheight function
float voltageToTempF(float voltage){
    float tempC = voltageToTempC(voltage);
    return tempCToTempF(tempC);
}


//a direct get temperature function for the tmp36 sensor using the prior voltage and temperature conversion functions
//simply input your pin number and output a float temperature!!!
float getTempF(int pinNumber){
    return voltageToTempF( getVoltage(pinNumber) );
}


////////////////////////////////////////////////////////////////////////////////
//                     End of function definitions
////////////////////////////////////////////////////////////////////////////////
