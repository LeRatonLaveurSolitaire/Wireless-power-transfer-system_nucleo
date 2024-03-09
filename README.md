# Wireless-power-transfer-system_nucleo

Implementation of the parameters identification technique on a NucleoG474RE dev board

## Phase shift modulation 

Timer 3 is used to generate two square waves at 85kHz, the ADC 3 is used with a potentiometer to set the phase shift

## Noise injection

The USER button is used to trigger the noise injection 

## Signal recording 

ADC 1 is used in diff mode with the DMA to mesure the current flowing through the primary coil during the noise injection.
Timer 1 is set to sample the current at 1MHz. 

## TODO :

* Finish current and voltage recording
* Implement frequency response extraction
* Test frequency response extraction
* Implement the neural network estimator
* Test the neural network estimator 
* Test the full identification 
