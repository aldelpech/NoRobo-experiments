=== Test 2 -bluetooth Arduino ===
Contributors: Anne-Laure
Power on and off a led connected to an arduino with a bluetooth hc-05 or hc-06 element
Runs only with the android app Joystick Bluetooth Commander Version 5.2


== DESCRIPTION ==
* To use : connect the smartphone to the hc-06 / hc-05 bluetooth receptor (1234)
* Button #1 controls pin #13 LED (on the arduino board)
* button #2 : controls pin #7 connected to white led and 330 ohm resistor
* Data1 and data2 fields returns dummy data 
* Data 3 returns a variable named displayStatus : it's value depends on the buttons pressed


== NOTES ==
* Joystick Bluetooth Commander Version 5.2 is an android App
* the arduino / hc-06 or hc-05 / led connexion are as shown in "schema-connexions.pdf"
* if you want to run both serial connexions (PC and bluetooth), RX and TX of the bluetooth
* module must be connected to other pins than rx and tx of the arduino (here pins # 2 & 3)

* this works perfectly on an arduino. 
* For some reason we were not able to run it on a gimbal GCC board. 

== Changelog ==

