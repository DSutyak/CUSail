CU Sail
Cornell University Autonomous Sailboat Team

Short Term Navigation Code
Version 5.1

Last Updated: 10/18/2016

This code is uploaded to the microcontroller of the SailVane.
This code is intended for use with the Arduino Due, but could be adapted to be used with other microcontrollers.

-------------------------------------------------------------------------------
Compile/Upload Instructions:

0. Download/update the Arduino IDE
1. Install drivers for the Arduino Due in the Arduino IDE. You can do so by 
   going to Tools -> Boards -> Board Manager, and searching for the appropriate drivers
2. Ensure ShortTermNavigation.ino, sensors.h, sensors.cpp, navigation.h,
   navigation.cpp, TinyGPS++.h and TinyGPS++.cpp are in a folder named "ShortTermNavigation"
3. Open "ShortTermNavigation.ino". All files except TinyGPS++.cpp and
   TinyGPS++.h should open in tabs within the Arduino IDE. If not you can
   go to Sketch -> Add File... to add the missing files
4. Select the appropritate Board and Port by going to Tools. Using
   the Arduino Due, make sure to use the Programming Port and not the Native USB
5. Click the *check mark* or go to Sketch -> Verify/Compile, to compile
   the code
6. Click the *right pointing arrow* or got to Sketch -> Upload, to
   upload code. The board should now run the code every time it receives power

-------------------------------------------------------------------------------

Noteworthy details/Changes since previous iteration:

- The Navigation Algorithm is a work in progress. The boat is not capable of executing any maneuvers 
  like Jibing and Tacking
- The boat direction is now averaged for 20 readings
- The algorithm has been slightly modified for when the boat is facing right. Updates for when the boat is 
  facing left is upcoming.

