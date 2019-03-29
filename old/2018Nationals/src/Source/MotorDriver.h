#ifndef MOTORDRIVER_H_INCLUDED
#define MOTORDRIVER_H_INCLUDED

namespace MotorDriver {
    void init(int fwidth, int fheight, int ldist);

    void update(double orientation, bool lineTest[4], double goalPos, float xval, float yval);
    void direction(double inangle);
    void setSpeed(int speed);
    void stop();

    int getMaxSpeed();

    double correct(double speed);
}   // namespace MotorDriver


#endif
