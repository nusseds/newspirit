/* 
  *  JoyDrive = Joystick Drive
  * ------------
  *
  * Reads two analog pins that are connected to potentiometers representing two
  * axis of a joystick. One axis represents steering angle, one axis represents
  * velocity. Each is normalized within a range from -100 to 100, so the sign
  * represents direction and number represents magnitude as a percentage.
  * 
  */

#ifndef joydrive_h
#define joydrive_h

#include "Arduino.h"
#include <ros.h>
#include <newspirit/SteerDrive.h>

#define STEER_MID 500
#define VEL_MID 520
#define ADC_MID 512
#define DEAD_ZONE 3

#define INVERT_STEERING false
#define INVERT_VELOCITY false

#define MAX_DELTA_STEERING 10
#define MAX_DELTA_VELOCITY 15
//using namespace ros
//char topic[] ="newspirit/cmd/steerDrive";

class JoyDrive
{
  public:
    JoyDrive();
    void init(ros::NodeHandle &nh);
    int getSteering();
    int getVelocity();

  private:
    int normalized(int raw, int midpoint, bool invert);
    int constrained(int normalized, int previous, int maxDelta);
    void messageCb(const newspirit::SteerDrive& msg);

    ros::Subscriber<newspirit::SteerDrive,JoyDrive> DriveSteer;
    
    int _steeringPin;
    int _velocityPin;

    int steeringRaw;
    int velocityRaw;
    
    int _previousSteering;
    int _previousVelocity;
};
#endif // joydrive_h
