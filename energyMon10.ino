/* energyMon10.ino UDP energy data server
 * Copyright (C) 2014, 2015  peter cowley
 * *********************************************************************
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * for a discussion of this software see:
 * https://community.spark.io/t/open-energy-monitor-port/3166
 * and
 * http://openenergymonitor.org/emon/
 *
 * ********************************************************************
 *
 * uses EmonLibPhot which is a slightly modified version of
 * openenergymonitor.org's emonLib
 *
 * Spark Connections:
 * ac voltage on pin A0 using 9v ac transformer divided 11:1 with
 *            100k/10k resistors
 * ac current on pin A1 from SCT-013-030 clip on sensor on mains supply tail
 * these are wired as described at: http://openenergymonitor.org
 *
 * In addition to emonlib functions, the monitor has interrupts on:
 *  pin D3 - reed switch attached to gas meter pull down to ground //was D0 on Core
 *  pin D1 - photoresistor on generator meter LED on meter pulls pin to ground
 *  pin D2 - photoresistor on domestic meter registers total domestic load
 *           but does not indicate whether importing or exporting
 *
 * All digital pins are pulled high with 100k resistors and decoupled with
 * 100nF ceramic capacitors
 * ***************************************************************************
 * The software is a UDP server
 * it responds to requests on port 5210 this could be any unused port number
 * the program loops continuously until there is a request when it makes
 * the measurements and returns them - having the client set the measurement
 * time avoids missing readings
 * Output is a string containing:
 *   timeSinceLastRequest/1000 in secs
 *   emon1.Vrms volts
 *   emon1.Irms amps
 *   emon1.realPower/1000.0 in Kw
 *   emon1.powerFactor -1 all export to +1 all import
 *   powerGas kW
 *   powerGen kW - from flash counting
 *   powerExp kW - from flash counting
 *   crossCount - number of mains cycles sampled
 *
 * because gas interrupts are widely spaced they are accumulated over
 * 20 (=NLOOPGAS) requests
 *
 *****************************************************************************
 * History
 *  v0.1 10/2/14
 *  v0.3 12/2/14 added reeds witch and photoresistor interrupts
 *  v0.4 12/2/14 added original Arduino PVgas.ino analysis
 *               to calculate cumulative and instantaneous power
 *               on the fly
 *  v1.0 19/2/14 include flag to indicate unread data output
 *               deleted in v1.1 when made a UDP SERVER
 *  tends to oscillate between adjacent values .24 - .48 0r .96 - 1.08
 *  because of the low number of flashes per LOOPT at low powers
 *  maybe note last nFlash and use intermediate value if delta is only 1?
 *  v1.1 don't update every 15 secs but every time polled
 *    this ensures that data are up to date and
 *    synchronised with external clock
 *    Everything goes inside parse packet loop
 *    Add reed relay chatter check. If powerGas > 40kW set to zero (normal max 32kW))
 *  v1.2 11/3/14 send waveform data - runs with powerControl2_0.py
 *  v2_0 13/3/14 tidy up and test
 * renamed EnergyMon
 * v1.0 adapted for Particle Photon
 * D0 cant be used as interrupt so use D3 instead
 * use port 5210 so development can parallel Core version
 * Photon trials has IP address 192.168.1.106
 * Photon dot            192.168.1.107
 * Core black 2          192.168.1.104
*****************************************************************************/

#include "application.h"
#include "EmonLib.h"

// set up an instance of EnergyMonitor Class from SemonLib
EnergyMonitor emon1;


// variables to convert flashes to kW
const long  FLASHKWH = 3600;    // 1 flash per sec is this many watts
const float TICKKWH = 400000.0; // 1 gas switch per sec is this many watts
const int   NLOOPGAS = 20;      // check gas every few loops 5 minutes for 15sec query

unsigned long currentTime;      // loop timer to keep UDP alive
unsigned long previousPoll;     // time of last request
unsigned long timeSinceLastRequest; //elapsed time since last request
int gasCount = 0;               // count number of times round loop since last gas update

// variables for UDP communications
UDP udp;
char UDPinData[64];
char UDPoutData[1024];           //128 bytes each of power, V wave and I wave
unsigned long portRead;         //last socket read
unsigned int localPort = 5212;  //reserved for incoming traffic
int packetSize;

// variables for interrupts
const long DEBOUNCE = 200;

int gasPin = D3;      //D3 for Photon D0 for Core
int genPin = D1;
int expPin = D2;

int ledPin = D7;

volatile unsigned long lastGas; //time since last flash for debounce
volatile unsigned long lastGen;
volatile unsigned long lastExp;

volatile int nGas = 0;    //number of flashes
volatile int nGen = 0;
volatile int nExp = 0;

volatile long cumGas = 0; //cumulative number of flashes
volatile long cumGen = 0;
volatile long cumExp = 0;

float powerGas;           //power values
float powerGen;
float powerExp;

int gasVal = 0;           //copy of number of flashes for small delta
int genVal = 0;           //so that adjacent measurements can be averaged
int expVal = 0;
float avFlash; //temporary storage for average of two adjacent nGen etc.

///////////////////////////////////////////
// interrupt function prototypes
void gasInt(void);
void genInt(void);
void expInt(void);

///////////////////////////////////////////
void setup() {

    udp.begin(localPort);
    portRead = millis(); //when port was last read
    previousPoll = portRead;

//check for Photon
#if PLATFORM_ID==6
    setADCSampleTime(ADC_SampleTime_112Cycles);//explore effect 480= 50pts per half cycle
#endif
    //Spark.variable("input", &input, STRING);
    //Spark.variable("results", &output, STRING);

    emon1.voltage(0, 250.0, 1.5); //initialise emon with pin, Vcal and phase
    emon1.current(1, 30);         //pin, Ical correct at 1kW

    pinMode(gasPin, INPUT);
    pinMode(genPin, INPUT);
    pinMode(expPin, INPUT);
    pinMode(ledPin, OUTPUT);
    attachInterrupt(gasPin, gasInt, RISING);
    attachInterrupt(genPin, genInt, RISING);
    attachInterrupt(expPin, expInt, RISING);
    lastGas = previousPoll;
    lastGen = previousPoll;
    lastExp = previousPoll;
    //clear the buffer
    memset(&UDPoutData[0], 0, sizeof (UDPoutData));

    digitalWrite(ledPin, LOW);
}

///////////////////////////////////////////
void loop() {

    currentTime = millis();
    //check for timeout and reset communications if necessary
    if (currentTime - previousPoll > 60000UL){
      System.reset();
//      WiFi.disconnect();
//      delay(1000);
//      Spark.connect();
//      delay(10000);
//      udp.stop();
//      delay(10000);
//      udp.begin(localPort);
//      previousPoll = currentTime;
    }

    // check whether there has been a request to the server and process it
    packetSize = udp.parsePacket();
    if (packetSize) {

        timeSinceLastRequest = currentTime - previousPoll;
        previousPoll = currentTime;
        // read the packet into packetBufffer
        udp.read(UDPinData, 64);
        // prepare power data packet
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        // update emon values
        emon1.calcVI(20, 1600);
        // now get values from meter flashes
        // the interrupt routines set nGas, nExp and nGen
        // first deal with the export meter flashes
        avFlash = nExp;
        if (abs(nExp - expVal) == 1) { //interpolate between small changes
            avFlash = (nExp + expVal) / 2.0;
        }
        powerExp = (float) FLASHKWH * avFlash / (1.0 * timeSinceLastRequest);
        if (nExp == 0) { //no flashes since last request so use emon value
            powerExp = emon1.realPower / 1000.0;
        }
        else if (emon1.powerFactor < 0) {
            powerExp *= -1.0; //use PF to add correct sign to meter value
        }
// note - you can get accurate and remarkably reliable import/export estimates
// by correlating the last 5 readings. Not implemented here but Arduino code
// available if anyone wants it.
        expVal = nExp; // remember number of flashes for next time
        nExp = 0;      //reset interrupt counter

        // now deal with PV meter flashes
        avFlash = nGen;
        if (abs(nGen - genVal) == 1) {//interpolate between small changes
            avFlash = (nGen + genVal) / 2.0;
        }
        powerGen = (float) FLASHKWH * avFlash / (1.0 * timeSinceLastRequest);
        genVal = nGen;
        nGen = 0;
        // Prepare diagnostic data for Spark variable
        //sprintf(input, " %2d, %2d, %.1f, %.2f ",\
                        nGen,genVal,avFlash,powerGen);
        // now deal with gas ticks of the reed switch
        // only update gas every NLOOPGAS loops (20 = 5min as ticks are slow
        gasCount++;
        if (gasCount == NLOOPGAS) {
            gasCount = 0;
            gasVal = nGas;
            powerGas = TICKKWH * nGas / (1.0 * NLOOPGAS * timeSinceLastRequest);
            nGas = 0;
            if (powerGas > 40) {//trap chatter if meter stops mid switch
                powerGas = 0;
            }
        } //end of slow gas calculation

        digitalWrite(ledPin, LOW); //set high by meter flash

        // we have finished calculating powers so put into a string for the UDP packet
/* **************************** This was a work around bad sprinff() ***********
        sprintf(UDPoutData, "%s %s %s %s %s %s %s %s %4d \n", \
          String(timeSinceLastRequest/1000.0,2).c_str(), String(emon1.Vrms,2).c_str(),String( emon1.Irms,2).c_str(),  \
          String(emon1.realPower/1000.0,2).c_str(), String(emon1.powerFactor,2).c_str(), String(powerGas,2).c_str(), \
          String(powerGen,2).c_str(), String(powerExp,2).c_str(),emon1.crossCount);
*/
        sprintf(UDPoutData, "%.2f %4d %4d %.2f %.2f %.2f %.2f %.2f %4d \n", timeSinceLastRequest/1000.0, \
        emon1.Vrms, emon1.Irms, emon1.realPower/1000.0, emon1.powerFactor, powerGas, \
        powerGen,  powerExp, emon1.crossCount);
        //and add the waveform arrays to the string
        for (int i = 0; i<256; i++){
             UDPoutData[256+i]=emon1.Vwaveform[(emon1.numberOfSamples+i+1)%256];
             UDPoutData[512+i]=emon1.Iwaveform[(emon1.numberOfSamples+i+1)%256];
             // offset by the number of samples so that we get the last 128
        }
        udp.write((unsigned char*)UDPoutData,768);
        udp.endPacket();
        //clear the buffer for next time
        memset(&UDPoutData[0], 0, sizeof (UDPoutData));
    }//finished writing packet
}//end of loop

///////////////////////////////////////////
void gasInt() {
    unsigned long thisTime;

    thisTime = millis();
    if ((thisTime - lastGas) > DEBOUNCE) {
        lastGas = thisTime;
        nGas++;
        cumGas++;
    }
}

void genInt() {
    unsigned long thisTime;

    thisTime = millis();
    if ((thisTime - lastGen) > DEBOUNCE) {
        lastGen = thisTime;
        nGen++;
        cumGen++;
    }
}

void expInt() {
    unsigned long thisTime;

    thisTime = millis();
    if ((thisTime - lastExp) > DEBOUNCE) {
        lastExp = thisTime;
        nExp++;
        cumExp++;
        digitalWrite(ledPin, HIGH);
    }
}
