#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

#define CLAW_CLOSE_DISTANCE    16 // Distance in arbitrary units

enum claw_pos {OPEN = 0, CLOSED = 1};
claw_pos claw_status;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *claw_motor = AFMS.getMotor(4); // The motor for the claw

/*!
 * \fn pick_block(int speed)
 * \brief runs the motors to pick the block
 * \param speed speed of grabber (0-255) 
*/
void pick_block(int speed) {
    if (claw_status == OPEN) {
        claw_motor->run(FORWARD);
        claw_motor->setSpeed(speed);
        delay(CLAW_CLOSE_DISTANCE / speed);
        claw_motor->setSpeed(0);
    } else {
        // serial.ln("Claw already closed");
    }
}

/*!
 * \fn release_block(int speed)
 * \brief runs the motors to release the block
 * \param speed speed of grabber (0-255) 
*/
void release_block(int speed) {
    if (claw_status == CLOSED) {
        claw_motor->run(BACKWARD);
        claw_motor->setSpeed(speed);
        delay(CLAW_CLOSE_DISTANCE / speed);
        claw_motor->setSpeed(0);
    } else {
        // serial.ln("Claw already open");
    }  
}