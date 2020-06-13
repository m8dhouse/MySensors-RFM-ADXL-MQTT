# MySensors-RFM-ADXL-MQTT

/* 
author: https://github.com/m8dhouse 
date: 13/6/2020 

License: https://choosealicense.com/licenses/gpl-3.0/

battery powered Arduino Pro mini leds + AM117 removed
ADXL345 on i2c RFM68W radio with helical coil antenna 
MySensors 8266 GW with link to MQTT server 4 positions to control light intensity (1,2,3,4) 
2 positions on cube with OFF (5, 6)  */  

/* sleep mode stuff 
based on work from TNTS https://forum.arduino.cc/index.php?action=profile;u=720419 
https://forum.arduino.cc/index.php?topic=486562.0  
*/


Not perfect though :-) Probably lot's of things I could improve. I don't program Arduino's often so ...
Before adding MySensors and the RFM the Arduino was going to sleep nicely.
Now it's not working as it should. Still need to add the sleepmode for the RFM.
The often used lowpower library or the sleep function in MySensors don't work properly with the ADXL345. 
The IRQ examples you can find for the ADXL345 - I couldn't get any of them to work to wake up the Arduino. 
The only example I found that worked is the one I used using the basic AVR functions.

I just wanted to created a proof of concept. 

