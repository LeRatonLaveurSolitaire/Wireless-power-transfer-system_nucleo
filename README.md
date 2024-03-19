# Wireless-power-transfer-system_nucleo

Implementation of the parameters identification technique on a NucleoG474RE dev board
A shield was made for this purpuse and can be found here : https://oshwlab.com/thomasboulanger2503/shield-wpt-nucleo-g474re

## Phase shift modulation 

Timer 3 is used to generate two square waves at 85kHz, the ADC 3 is used with a potentiometer to set the phase shift

## Noise injection

The USER button is used to trigger the noise injection 

## Signal recording 

ADC 1 is used in diff mode with DMA1 to mesure the current flowing through the inverter during the noise injection.
ADC 2 is used in diff mode with DMA2 to mesure the voltage at the output of the inverter during the noise injection.
Timer 1 is set to sample the current at 1MHz. 

Once recorded and processed, the estimated parameters are printed on the Oled Display

