/*
Phoenix College Spring 2017 Launch Code






HEADER DEFINITION:
example: "IMU Raw,IMU euler,Temp0(F),Temp1(F),CO2,Geiger(cpm),GPS,millis(ms),APRS"

IMU Raw     =raw output from all IMU sensors (this will be redefined after implementing IMU to multiple positions)
IMU euler   =euler angle output from IMU (will be updated in the future to multiple positions 3 axis)
Temp0(F)    =TMP36 sensor data in fahrenheit for the TMP36 on the inside of the payload
Temp1(F)    =TMP36 sensor data in fahrenheit for the TMP36 on the outside of the payload
CO2         =output reserved for CO2 sensor. will be updated when implemented
Geiger(cpm) =general geiger counter in counts per minute. Will be updated when we implement more geiger counters
GPS         =gps output, will be updated with more placements later. Currently planning to capture GPGGA type NMEA GPS sentences.
millis(ms)  =the duration that the current program has been running in milliseconds. If we have no GPS fix, we have no sense of time. This uses the arduino local clock to tell how much time has passed, and as indication of system failures
APRS        =placeholder for data from APRS. (this should just be a transmitter, but we should also be able to get location from it as well... maybe...)

*/









////////////////////////////////////////////////////////////////////////////////
//   Declare globals like pin numbers or constants/volatile variables here.
////////////////////////////////////////////////////////////////////////////////

int temp0Pin = A0;             //I'm writing A0 because you guys seem to use that. I use just 0. Both are valid.
int temp1Pin = A1;

const unsigned int geigerGeneralPin = 2; //assigns digital input pin 2 to the geiger code
// int geigerAlphaPin = ;   //when these are bought and tested, add these pins
// int geigerBetaPin = ;
// int geigerGammaPin = ;
// Global variables
   
// Declared volatile because two threads of execution are using it.
// Value is initially set to zero because there are no counts.   
   volatile unsigned int gc_counts = 0;
   
// Function for interrupt 
   // gc_counts is increased by 1 
   // every time function called.
   // void loop will reset when written memory 
   void gc_interrupt(){ 
     gc_counts++;
   }   
char delimiter = ',';   //used for seperating sensor values in the logging file

////////////////////////////////////////////////////////////////////////////////
//                          End of global variables
////////////////////////////////////////////////////////////////////////////////













////////////////////////////////////////////////////////////////////////////////
//                          Start of setup
////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);

    Serial.println( F("IMU Raw,IMU euler,Temp0(F),Temp1(F),CO2,Geiger(cpm),GPS,millis(ms),APRS") );  //the F() function stores the entire string literal only in flash memory, which saves a bit of program space. Better to add it now, then to have issues later and take a long time to diagnose and fix. This is preventative

// Sets up geigerGeneralPin as input
   pinMode(geigerGeneralPin, INPUT);
  
// Sets up the interrupt to trigger for a rising edge
// geigerGeneralPin (2) corresponds to the 5th interrupt gc_intnumber  
// gc_interrupt is the function we want to call once we detect an interrupt
   attachInterrupt(digitalPinToInterrupt(geigerGeneralPin), gc_interrupt, RISING);
   
 
}//end of setup
////////////////////////////////////////////////////////////////////////////////
//                          End of setup
////////////////////////////////////////////////////////////////////////////////














////////////////////////////////////////////////////////////////////////////////
//                          Start of loop
////////////////////////////////////////////////////////////////////////////////
void loop() {


    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of IMU Raw
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("IMU Raw");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of IMU Raw
    ////////////////////////////////////////////////////////////////////////////////





    ////////////////////////////////////////////////////////////////////////////////
    //                          Start of IMU euler
    ////////////////////////////////////////////////////////////////////////////////
    Serial.print("IMU euler");
    Serial.print(delimiter);
    ////////////////////////////////////////////////////////////////////////////////
    //                          End of IMU euler
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
    //                          Start of Geiger(counts per 5 seconds)
    ////////////////////////////////////////////////////////////////////////////////
    unsigned long particleCount1 = gc_counts;//taking initial radiation sample
    delay (5000);//waiting 5 seconds
    unsigned long particleCount2 = gc_counts;//taking second radiation sample
    unsigned long deltaCount = particleCount2 - particleCount1;//since our radiation pulses area a running count we need to take the difference of two samples given a known time lapse to deterimine the counts per 5 seconds.
    Serial.print(deltaCount);//prints counts over a 5 second span and prints them/sends them to the datalogger
    Serial.print(delimiter);// POST Geiger Counter (GC) test code






  


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
