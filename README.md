# D-OSHADOW
SHADOW Control System for Mr. Baddeley's D-O version2.  This is an ALPHA release.  
Do not use as your droid could run away and hit things.  

**UPDATE**
The runaways seem to tbe caused by electromagnetic interference (EMI). There needs some EMI shielding on the motor wires.  The signals generated at some speeds of the motors can actually jam the Bluetooth signal. I am working on a solution.  Stay tuned.

This project is in progress and is being posted in the hopes others will contribute.
You will need the ServoEasing library.  It is available in the Arduino Library Manager.
You will have to edit the .h file in the ServoEasing Library to enable the PCA9685 expander.

Here is the current hardware
- Arduino Mega 2560  

- Arduino USB Shield
  
- Cytron MDD10A Motor driver
 
- DFPlayer Mini
  
- 6050 IMU (Inertial Measurement Unit)
  
- Pololu 20.4:1 HP 12V 25mm DC Motors (No encoder)
  
- PWM 9685 Servo Expander
  
- HiTech HS-645MG (MainArmServo)
  
- HiTech HS-85MG  (HeadNodServo)
  
- HiTech HS-65HB  (HeadTiltServo
  
- HiTech HS-65HB  (HeadTrun Servo)
