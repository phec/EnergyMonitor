/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due & Spark Core/Photon)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

#ifndef EmonLib_h
#define EmonLib_h

#include "application.h"

#include "math.h"

// to enable 12-bit ADC resolution on Arduino Due, 
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.

#define ADC_BITS    12


#define ADC_COUNTS  (1<<ADC_BITS)


class EnergyMonitor
{
  public:

    void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
    void current(unsigned int _inPinI, double _ICAL);

    void calcVI(unsigned int crossings, unsigned int timeout);
    double calcIrms(unsigned int NUMBER_OF_SAMPLES);
    void serialprint();

    //Useful value variables  Particle - make floats not double
    float realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
    unsigned int numberOfSamples;
    unsigned int crossCount;
    uint8_t Vwaveform[128];
    uint8_t Iwaveform[128];
  private:

    //Set Voltage and current input pins
    unsigned int inPinV;
    unsigned int inPinI;
    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    float VCAL;
    float ICAL;
    float PHASECAL;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
	int sampleV;   //sample_ holds the raw analog read value
	int sampleI;                      

	float lastFilteredV,filteredV;                   //Filtered_ is the raw analog value minus the DC offset
	float filteredI;
	float offsetV;                          //Low-pass filter output
	float offsetI;                          //Low-pass filter output               

	float phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

	float sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

	int startV;                                       //Instantaneous voltage at start of sample window.

	bool lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.


};

#endif
