#include "PID_lib.h"

float goal;
double threshold;
double startTime;
double pastTime;

/* Global Variables */
// Encoder variables
volatile double count = 0;   // encoder step count, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
const int encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
double prev_count = 0;
double curr_count = 0;
const double counts_per_rev = 464.64;
unsigned long t_duration = 0;
unsigned long curr_time = 0;

// PID variables
const double ref_angular_pos = 20;    // 20 revolutions from start
const double ref_angular_speed = 12;  // 12 revolutions per second
double curr_angular_pos = 0;
double curr_angular_speed = 0;
double curr_error_ang_pos = ref_angular_pos;       // initialize the error signals
double curr_error_ang_speed = ref_angular_speed;
double prev_error_ang_pos = ref_angular_pos;
double prev_error_ang_speed = ref_angular_speed;
double curr_error = 0;
double prev_error = 0;
double total_error = 0;
double pid_p_term;
double pid_i_term;
double pid_d_term;
double pid_output;
bool is_tracking_ang_pos = 0;
double kp = 0;
double ki = 0;
double kd = 0;

// general purpose variables
int is_first_loop = 1;
double prev_pwm_val = 0;
double curr_pwm_val = 0;
const double error_threshold_pos = 0.001;
const double error_threshold_speed = 0.001;
double scaleFactor = 0;


//Return values
bool forward = 0;
bool backward = 0;
double PWMValue;


PID::PID(float goalC, double PC, double IC, double DC, double thresholdC,
    double startTimeC, bool pos, float current, int ang_pos,
    double ratio)
{
    is_tracking_ang_pos = ang_pos;
    goal = goalC + current;
    kp = PC;
    ki = IC;
    kd = DC;
    threshold = thresholdC;
    startTime = startTimeC;
    pastTime = 0;
    is_tracking_ang_pos = pos;
    scaleFactor = ((double) 255 / goal) * ratio;

}


void PID::execute(float current, double instTime)
{
    // Read and store the current encoder measurement
    // NOTE: when accessing the encoder count, prevent interrupts from modifying the variable
    curr_count = current;

    // obtain the time duration of this past loop
    t_duration = (instTime - pastTime)*.001;
    curr_time = instTime*.001;

    // calculate the current motor angular position & velocity
    curr_angular_pos = curr_count / counts_per_rev;                       // unit: # of positive revolutions from start
    curr_angular_speed = (curr_count - prev_count) / t_duration;   // unit: counts per second //CHANGE?
    curr_angular_speed = curr_angular_speed / counts_per_rev;             // unit: revolutions per second

    // obtain the current error in angular position & velocity, and store the last error
    curr_error_ang_pos = goal - curr_angular_pos;
    curr_error_ang_speed = goal - curr_angular_speed;

    /* PID control starts here!! */
    // First determine what we want to track: angular pos or speed, and set error signal accordingly
    if (is_tracking_ang_pos == 1)
    {
        curr_error = curr_error_ang_pos;
        prev_error = prev_error_ang_pos;
    }
    else
    {
        curr_error = curr_error_ang_speed;
        prev_error = prev_error_ang_speed;
    }
    total_error += curr_error;

    // Next, obtain the three terms of the PID controller output
    pid_p_term = kp * curr_error;
    pid_i_term = ki * total_error;
    pid_d_term = kd * (curr_error - prev_error) / (t_duration); //CHANGE?

    // Finally, set the PID output depending on if this is the very first loop
    if (is_first_loop == 1)
    {
        pid_output = pid_p_term;
    }
    else
    {
        pid_output = pid_p_term + pid_i_term + pid_d_term;
    }

    // Last step: convert PID output to PWM signal, send it to the motor together with the correct spinning directions
    if (is_tracking_ang_pos == 1)
    {
        if (absolute(curr_error) <= error_threshold_pos)
        { PWMValue = 0; }
        else
        {
            if (curr_error > 0) {
                forward = 1;
                backward = 0;
            }
            if (curr_error < 0) {
                forward = 0;
                backward = 1;
            }
            PWMValue = constrain(absolute(pid_output) * scaleFactor, 0.0, 255.0);
        }
    }
    else
    {
        if (ref_angular_speed > 0) {
            forward = 1;
            backward = 0;
        } else {
            forward = 0;
            backward = 1;
        }
        PWMValue = constrain(absolute(pid_output), 0.0, 255.0);
    }

    prev_pwm_val = PWMValue;
    pastTime = curr_time;
    prev_count = curr_count;
    prev_error_ang_pos = curr_error_ang_pos;
    prev_error_ang_speed = curr_error_ang_speed;
    is_first_loop = 0;
}

double PID::getPWMValue() {
    return PWMValue;
}

bool PID::getForward() {
    return forward;
}

bool PID::getBackward() {
    return backward;
}

bool PID::getOutput() {
    return pid_output;
}

const double constrain(const double x, const double a, const double b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

const double absolute(const double x) {
    if(x < 0) {
        double r = x * -1;
        return r;
    } else
        return x;
}

