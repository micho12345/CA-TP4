/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>
const unsigned long interval = 10000; // intervalo en us

#include "lowpass.h"
#include "highpass.h"

#include <FIR.h>

FIR<double, FILTER_TAP_NUM_LP> fir_lp;
FIR<double, FILTER_TAP_NUM_HP> fir_hp;

void setup() {
  // put your setup code here, to run once:
  Timer1.initialize(interval);   //  conversión a us
  Timer1.attachInterrupt(timerCallback);

  fir_lp.setFilterCoeffs(filter_taps_lp);
  fir_hp.setFilterCoeffs(filter_taps_hp);

  fir_lp.setGain(1.0);
  fir_hp.setGain(1.0);

  // Set Input pins

  // Set Output pins


}

void loop() {
  // put your main code here, to run repeatedly:

}

void timerCallback()
{
  // Run the filters
  int sample = AnalogRead();
  double sample_v = map(sample, 0, 255, -2.5, 2.5);
  fir_lp_o = fir_lp.processReading(sample);
  fir_hp_o = fir_hp.processReading(sample);

  analogWrite(fir_lp_o + fir_hp_o);
}