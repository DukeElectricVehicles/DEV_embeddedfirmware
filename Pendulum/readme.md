# Pendulum Data Collection Rig

This is the code for the pendulum data collection rig

## Hardware

This rig requires 2 components:
* the sender board - it has a battery duck-taped to a teensy/IMU/nRF board.  This board goes on the pendulum.
* the receiver board - it's just a teensy on an nRF shield.  This board plugs into your computer

## Firmware

The sender board should already be flashed, but for reference, it uses the "Pendulum.ino" code.  Change the pound-defines at the beginning of the file so that the defines are "SENDER" and "RECEIVERx".

The receiver board is sometimes used for different purposes so, if you're not getting what you expect, it's worth reflashing it with the same Pendulum.ino code but using the pound-defines "SENDERx" and "RECEIVER".

## Startup Procedure

The receiver can just be plugged into your computer.

The sender is more tricky - it needs to be "calibrated" upon startup.  AS SOON AS YOU PLUG IN THE BATTERY, MAKE SURE THE DEVICE IS COMPLETELY MOTIONLESS.

To plug in the battery, find the two pins sticking out of the board.  One should say "VIN" near it and the other one doesn't have any text next to it (but that's the GND pin).  Connect the little 4-pin connector to these two pins and just make sure that the VIN pin gets a higher voltage than the GND pin - if in doubt, just match up the black wire on the battery connector with the GND pin on the board.

## Data Collection

Connect the receiver to your computer and open up any serial monitor (you don't need Teensyduino installed).  The columns are coded as follows:

    Serial.print(data.time_ms); Serial.print('\t');
    Serial.print(data.AngleX);  Serial.print('\t');
    Serial.print(data.AngleY);  Serial.print('\t');
    Serial.print(data.AngleZ);  Serial.print('\t');
    Serial.print(data.AccX);    Serial.print('\t');
    Serial.print(data.AccY);    Serial.print('\t');
    Serial.print(data.AccZ);    Serial.print('\n');