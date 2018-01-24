/*
 * Project BluzTemperatureSensor
 * Description: BluzDK with light sensor, TMP36, door sensor on interrupt, and vibration motor
 * Author: Electrophoenix
 * Date: 1/28/2018
 */

 SYSTEM_MODE(AUTOMATIC);

// find way to do this without keeping the cpu on for an entire

//10-bit ADC, max value is 1023
int ADCbit = 1023;

// set up MQTT data
String myID;

//update interval
int updateInterval = 15*60*1000;        // update every 15 seconds

// Define pins
int lightPin = A0;
int ldrPowerPin = D5;
int tempPin = A1;
int vibPin = D0;
int doorPin = D6;

// Define input variables coming from these pins

// bat variables
float supplyVoltage = -1;

// light variables
//double light = -1;
int lightResistor = 5000;       // Resistance, in Ohms, of resistor in series with LDR
int ldrMaxResistance = 13000;   // Maximum resistance, in Ohms, of LDR in the dark
int ldrMinResistance = 800;     // Minimum resistance, in Ohms, of LDR in bright light
const int ldrArraySize = 5;
int ldrResistance[ldrArraySize];
double brightnessPct = 0;
int ldrArraySum = 0;
int readingIndex = 0;
/*double lightReading2;
double lightReading3;
double lightReading4;
double lightReading5;*/

// temp variables
double temp = -1;

// door sensor variables
volatile bool doorTrigger = false;
bool doorSensor = false;

// Trigger variables
double oldTemp = -1;
double oldBrightnessPct = -1;
bool timerTrigger = true;
bool tempTrigger = false;
bool lightTrigger = false;

// Timers trigger when to take readings
Timer processTimer(60*1000, sensorChecks);
Timer updateTimer(updateInterval, updateCallback);
Timer vibTimer(500, vibStop, TRUE);

int currentTime = 0;

void setup()
{
    // power saving measure - make all pins not floating
    for (int i = 0; i < 20; i++)
        pinMode (i, INPUT_PULLDOWN);

    //disable the status LED
    RGB.control(true);
    RGB.color(0, 0, 0);

    // Set up pins
    pinMode(lightPin, INPUT_PULLDOWN);
    pinMode(ldrPowerPin, OUTPUT);
    digitalWrite(ldrPowerPin, LOW);
    pinMode(tempPin, INPUT_PULLDOWN);
    pinMode(doorPin, INPUT_PULLDOWN);
    pinMode(vibPin, OUTPUT);
    digitalWrite(vibPin, LOW);


    // attach interrupt
    attachInterrupt(doorPin, doorSensorInterrupt, CHANGE);

    // initialize values
    doorSensor = digitalRead(doorPin);
    for(int n=1;n<ldrArraySize;n++){ // get an average for the light sensor
        lightReading();
        delay(10);
    }

    // Set up Particle Variables
    //Particle.variable("cuipo_light", light);
    //Particle.variable("cuipo_temp", temp);
    Particle.variable("updateFreq", updateInterval);
    Particle.variable("currentTime", currentTime);

    // Set up particle functions
    Particle.function("vibrate", vibrate); //connect function to activation in cloud
    Particle.function("updateFreq", updateFrequency);

    // prepare for  mqtt over local bluetooth
    myID = System.deviceID();
    //Particle.publish("Bluz DeviceID", myID);
    //memcpy(data,myID.c_str(),24);

    // announce presence
    btToMQTT(myID+"|online");

    processTimer.start();
    updateTimer.start();
}

void loop()                     // run over and over again
{
    if(doorTrigger||tempTrigger||lightTrigger||timerTrigger){
        // read door sensor
        doorSensor = digitalRead(doorPin);

        // build the json output
        String json = myID
            +"|{\"battery\":"+String(supplyVoltage)
            +", \"temperature\":"+String(temp)
            +", \"light\":"+String(brightnessPct)
            +", \"door\":"+String(doorSensor)
            +", \"triggers\":"
            + String(timerTrigger*1000+tempTrigger*100+lightTrigger*10+doorTrigger*1)
            +"}";

        // Convert to char array and send data to local MQTT broker
        btToMQTT(json);

        // Publish to Particle cloud as event
        //Particle.publish(MyID, json.substring(25), PRIVATE);

        // reset timer and trigger
        timerTrigger = false;
        lightTrigger = false;
        tempTrigger = false;
        updateTimer.reset();
        oldTemp = temp;
        oldBrightnessPct = brightnessPct;
        doorTrigger = false;
    }
    System.sleep(SLEEP_MODE_CPU);
}

// Update if the door opens or closes
void doorSensorInterrupt(){
    doorTrigger = true;
}

// If nothing else happens, update on a timed interval
void updateCallback(){
    timerTrigger = true;
}

// PARTICLE CALL: vibrate the motor for a certain amount of time
int vibrate(String length){
    digitalWrite(vibPin, HIGH);
    vibTimer.changePeriod(atoi(length));
    vibTimer.start();
    return 1;
}

// Stop the vibrating motor
void vibStop(){
    digitalWrite(vibPin, LOW);
}

// PARTICLE CALL: change update interval
int updateFrequency(String interval){
    updateInterval = atoi(interval);
    updateTimer.changePeriod(updateInterval);
    return 1;
}

// Send data to gateway for publishing to MQTT broker
void btToMQTT(String json){
    int length = json.length();
    char data[length] = {};
    memcpy(data,json.c_str(),length);       // copy string buffer to char array
    BLE.sendData((uint8_t*)data, length);   // Send data to gateway
}


// Update all the sensors and flip triggers
void sensorChecks(){
    // Update all the sensors

    // Noisy data: update after a short interval has elapsed
    // Clean changes: if the value stabilizes then send the new value

    // Read supply voltage moved to right leading function to prevent fluctuations
    //supplyVoltage = readSupplyVoltage()*3.6/ADCbit;

    // Read light sensor and average values over past 5 readings
    brightnessPct = lightReading();


    // Read temp sensor
    int tempReading = analogRead(tempPin);                      // converting that reading to voltage, for 3.3v arduino use 3.3
    double tempVoltage = (tempReading * 3.6 / ADCbit);          // converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)
    temp = (round((tempVoltage-0.5) * 100*10)/10) * 9/5+32;     // Fahrenheit

    // Triggers
    tempTrigger = abs(temp-oldTemp)>1;
    lightTrigger = abs(brightnessPct - oldBrightnessPct)>5;
}

// Update the LDR array and return a time-averaged reading
double lightReading(){
    digitalWrite(ldrPowerPin, HIGH);                                                // Turn on power to the photosensor
    ldrArraySum = ldrArraySum-ldrResistance[readingIndex];                          // remove the current value from the sum
    int rawVoltageReading = analogRead(lightPin);                                   // take a reading
    int rawSupplyVoltage = readSupplyVoltage();
    supplyVoltage = rawSupplyVoltage*3.6/ADCbit;
    ldrResistance[readingIndex]=rawVoltageReading*lightResistor/(rawSupplyVoltage-rawVoltageReading);    // replace the reading in the array
    ldrArraySum = ldrArraySum+ldrResistance[readingIndex];                             // update the same with the new value                                                                  // advance the array indexer
    brightnessPct = 100-(ldrArraySum/ldrArraySize-ldrMinResistance)*100/(ldrMaxResistance-ldrMinResistance);    // interpolate between high and low values
    digitalWrite(ldrPowerPin, LOW);                                                 // Turn off the power pin
    readingIndex++;
    if (readingIndex >= ldrArraySize){
        readingIndex=0;
    }
    return brightnessPct;
}
