# OSC_Clock

A stepper-driven clock controlled by OSC. This was created for a production of Rodgers and Hammerstein's Cinderella. It never made the stage because as it always does, time ran out. 

![Fritzing Image](images/Clock_bb.png)


## Parts
 * CPU: NodeMCU 12E
 * Stepper Driver: DRV8825 (https://www.pololu.com/product/2133)
 * RTC: DS3231 (https://www.adafruit.com/product/5188)
 * Stepper: NEMA 17 1.8 degree
 * Frame/gears: Laser-cut parts
 * Hardware:
   * 12x 3mm x 30mm hex head screws.
   * 12x 3mm nyloc nuts.
   * 3x 608z ball bearings
   * 40+mm of 8mm Al tube. 
   * 30+mm of 5mm fiberglass rod. 
 * IR LED + IR detector

## OSC commands

 * /clock/home - advance the clock hands to the "home" position as detected by the optical sensor. 
 * /clock/stop - stop the clock
 * /clock/now - advance the hands to the current time as set in the RTC. 
 * /clock/run - Run as a clock, keep in sync with the RTC time.
 * /clock/gear $gear - change the virtual gearing of the stepper. options (1,2,4,6,16,32)
 * /rtc/set - TBD
 * /rtc/get - TBD
