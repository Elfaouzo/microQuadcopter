# microQuadcopter

Ram consumption reduction. Mostly to improve datalogging possibilities.
All the motor/throttle related signals are PWM output controlled between 0-255. The idea is to change them from float to uint8.
