## PowerFilm Nordic Reference Design w/ E-peas AEM10941

This repository contains the schematic and board files for PowerFilm's Nordic Reference Design using E-peas AEM10941.

The schematic and board layouts are currently untested, but near completion.

# AEM10941 Configuration

This board has been designed so the user can configure the AEM10941 chip for their specific application. Documentation on what each configuration does can be found in the [AEM10941 datasheet](https://e-peas.com/wp-content/uploads/2020/04/DS_AEM10941.pdf).  

High and low voltages are output by default, although only the high voltage output is being used. The outputs can be disabled by cutting and soldering the corresponding solder bridge on the back of the board.

The initial configuration is CNFG[2:0]={0,1,1}, which allows the most energy to be stored in the 2 220uF capacitors on board while also outputing a suitable voltage for the NRF52832. The configuration state can be changed by cutting and soldering the corresponding solder bridges on the back of the board.

R1 through R4 correspond can be populated according to the AEM10941 datasheet if the user requires a custom mode. R5 and R6 are used to set the high voltage feedback, and R9 and R10 are used to set the cold start minimum voltage. All resistors <= R10 match the resistor numbers/location in the AEM10941 datasheet.

Holes for breakout pins have been provided to allow the user to access the status, attach an external battery (with or without a balance pin) and a primary battery, and have a ground point.

# Capacitors

By default two 1206 sized, 220uF capacitors have been populated on the front of the board. On the back of the board there is room for an additional five 1206 sized capacitors, as well as five 1210 sized capacitors. 

Footprints for Cap-XX and AVX supercapacitors are provided for applications requiring higher energy storage. The specific models of super capacitor are provided in the schematic.

**The balance output of the AEM10941 must be grounded when not in use and is connected to ground by default on this board. Be sure to cut the solder bridge if the balance output is required.**

If more energy storage than a supercapacitor is required, the BATT output can be connected to a rechargable battery providing the user configures the board to suit the battery.

# Nordic nRF52832

This board uses the nRF52832 SoC for its various Bluetooth operations. Attached to the nRF52832 is an LED, a button, 3 exposed GPIO pins, and a 6 pin Tag-Connect footprint. 
