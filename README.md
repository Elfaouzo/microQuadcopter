# microQuadcopter

This software is intended for particle Photon board. It is meant to be used within the particle web IDE.
The inputs of the system are described below:
  - IMU (via i2c).
  - Battery voltage reading (via analog input).
  - PPM RC receiver (via digital inputs).
  - TCP server (via onboard wifi module).
  
The outputs of the system are:
  - 4x PWM motor control (PWM outputs).
  - Status LED (digital output).