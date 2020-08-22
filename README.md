# EDTracker2
http://www.edtracker.org.uk

EDTracker2 sketches for Arduino IDE. These are subdivided by various hardware
types; make sure you choose the sketch appropriate for the hardware you're using
(motion sensor, for example). Place these in your Arduino IDE sketches folder
(typically in "My Documents/Arduino").

If you are using standard SparkFun Pro Micro hardware coupled with MPU-6050 or
MPU-9*50 MEMS sensors, there is no need for you to build from source if you
keep to the standard wiring format provided on www.edtracker.org.uk. The simplest
route to EDTracker ownership is to build the physical device following the
instructions on the website, and then simply download the EDTracker GUI and
flash your device from that. No need to faff around with source code, Git and
compiling!

NOTE: To make the build process easier, all the libraries and relevant hardware
settings are packaged up and available within the separate repo here :

https://github.com/brumster/EDTracker2_ArduinoHardware

You WILL need the additional libraries and modified Arduino core files provided in the
above repo; the sketches will not compile without them :)

EDTracker 2 sketches are currently as follows :

- EDTracker2_6050 - for use with Invensense MPU-6050 breakout boards (accel/gyro only)
- EDTracker2_9150 - for use with Invensense MPU-9150 breakout boards (accel/gyro/magnetometer)
- EDTracker2_9250 - for use with Invensense MPU-9250 breakout boards (accel/gyro/magnetometer)

Once built, you will need the EDTracker GUI software to calibrate and monitor -
this is available in binary form only at this stage, via
http://www.edtracker.org.uk/index.php/downloads

Tested/verified with Arduino IDE 1.8.5

Love and manly hugs!
The EDTracker Team...
