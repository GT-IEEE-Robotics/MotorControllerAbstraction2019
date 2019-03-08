#ifndef PID_h
#define PID_h
#include <cmath>

class PID
{
public:
    PID(float goalC, double PC, double IC, double DC, double thresholdC, double startTimeC, bool pos, float current, int ang_pos, double ratio);
    void execute(float current, double instTime);
    double getPWMValue();
    bool getForward();
    bool getBackward();
    bool getOutput();
};
const double constrain(const double x, const double a, const double b);
const double absolute(const double x);

#endif
