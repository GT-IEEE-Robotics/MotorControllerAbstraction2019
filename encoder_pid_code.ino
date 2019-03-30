// Encoder + PID codes: perform motor angular position & velocity tracking
// last revision: 02/26/2019

/* pin defintions */
#define PWMA (6)    // output PWM signal for motor A
#define PWMB (5)    // output PWM signal for motor B
#define ENC_A (2)   // Encoder Input A for motor A
#define ENC_B (3)   // Encoder Input B for motor A
#define ENC_C (8)   // Encoder Input A for motor B
#define ENC_D (9)   // Encoder Input A for motor B
#define AIN1 (10)
#define AIN2 (11)   // AIN1/2 for motor A
#define BIN1 (12)
#define BIN2 (13)   // AIN1/2 for motor B

/* Global Variables */
// Encoder variables
volatile int32_t count = 0;   // encoder step count, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int32_t prev_count = 0;
int32_t curr_count = 0;
const double counts_per_rev = 464.64;   // TO BE CHANGED WITH SPECIFIC MOTOR SPECS
unsigned long t_duration = 0;
unsigned long curr_time = 0;

// PID variables
double ref_angular_speed = 7;  // THIS IS THE DESIRED SPEED FOR MOTOR A, in rev/s
double ref_angular_pos = 20;   
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
double kp = 8;           // 8    NEED MORE TUNING TO REDUCE INITIAL OVERSHOOT & SETTLING TIME
double ki = 9;            // 9
double kd = 0.1;          // 0.1

// general purpose variables
int8_t is_first_loop = 1;
const int8_t is_tracking_ang_pos = 0;
const int8_t is_tracking_ang_speed = 1;
const double error_threshold_pos = 0.01;
const double error_threshold_speed = 0.01;
double pwm_output;
double pwm_scale = 500;

// TESTING PURPOSE
//double curr_sec = 0;
//double ref_angular_speed_sequence[] = {-8,-7,-6,-5,-4,-3,-2,0,2,3,4,5,6,7,8};
//int8_t ref_speed_seq_length = 15;
//int8_t curr_speed_seq_idx = 0;

void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(ENC_C, INPUT);
    pinMode(ENC_D, INPUT);

    Serial.begin(9600);      // open the serial port at 115200bps

    // attach the encoder interrupt function to changes on both encoder input pins
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr, CHANGE);

    // record the current time
    curr_time = millis();
}

void loop() {

    // TESTING: SEQUENCE OF REF SPEED, FIXED DURATION
    //if (millis() / 1000 - curr_sec > 2)
    //{
    //    if (curr_speed_seq_idx < ref_speed_seq_length)
    //    {
    //        curr_speed_seq_idx = curr_speed_seq_idx + 1;
    //        ref_angular_speed = ref_angular_speed_sequence[curr_speed_seq_idx];
    //    }
    //    else
    //    {
    //        ref_angular_speed = 7;
    //    }
    //    curr_sec = millis() / 1000;
    //}
    
    // wait for 0.2 sec between loops
    delay(200);
    
    // Read and store the current encoder measurement
    // NOTE: when accessing the encoder count, prevent interrupts from modifying the variable
    noInterrupts();
    prev_count = curr_count;
    curr_count = count;
    interrupts();

    // obtain the time duration of this past loop
    t_duration = millis() - curr_time;
    curr_time = millis();
    
    // calculate the current motor angular position & velocity
    curr_angular_pos = curr_count / counts_per_rev;                       // unit: # of positive revolutions from start
    curr_angular_speed = (curr_count - prev_count) / (t_duration*0.001);   // unit: counts per second
    curr_angular_speed = curr_angular_speed / counts_per_rev;             // unit: revolutions per second

    // obtain the current error in angular position & velocity, and store the last error
    prev_error_ang_pos = curr_error_ang_pos;       
    prev_error_ang_speed = curr_error_ang_speed;
    curr_error_ang_pos = ref_angular_pos - curr_angular_pos;
    curr_error_ang_speed = ref_angular_speed - curr_angular_speed;

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
    
    // check for the range of the current reference speed, and select proper set of PID parameters
    if (abs(ref_angular_speed) <= 3)
    {
        if (abs(curr_angular_speed) < 0.1)
        {
          kp = 5;
          ki = 6;        // this set neesd more tuning: speed start from 0 to a ref value less than 2
          kd = 0.1;
        }
        else
        {
          kp = 4;
          ki = 4.5;
          kd = 0.1;
        }
    }
    else
    {
        kp = 8;
        ki = 9;
        kd = 0.1;
    }

    // Next, obtain the three terms of the PID controller output
    pid_p_term = kp * curr_error;
    pid_i_term = ki * total_error;
    pid_d_term = kd * (curr_error - prev_error);

    // Finally, set the PID output depending on if this is the very first loop
    if (is_first_loop == 1)
    {
        pid_output = pid_p_term;
    }
    else
    {
        pid_output = pid_p_term + pid_i_term + pid_d_term;
    }

    // convert PID output to a PWM value scaled between 0 and 255
    //pid_output = pid_output * pid_scale_factor;
    
    // Last step: convert PID output to PWM signal, send it to the motor together with the correct spinning directions

    // Added part to make sure PID integrator (I term) doesn't accumulate error signals too much
    // if saturate, then reset
    if (abs(pid_output) > 255)
    {
        total_error = 0;
    }
    
    /* This part is for angular position tracking */
    if (is_tracking_ang_pos == 1)
    {
        if (abs(curr_error) <= error_threshold_pos)
        { analogWrite(PWMA,0); }
        else
        {
            if (curr_error > 0) { motorForward(); }
            if (curr_error < 0) { motorBackward(); }
            pwm_output = constrain(abs(pid_output), 0, 255);
            analogWrite(PWMA, pwm_output);
        }
    }
    /* This part is for angular speed tracking */
    else
    {
        if (ref_angular_speed > 0) { motorForward(); }
        else { motorBackward(); }
        pwm_output = constrain(abs(pid_output), 0, 255);
        analogWrite(PWMA, pwm_output);
    }
    
    is_first_loop = 0;
    

    // print the current motor status
    Serial.print("Ang Pos: ");
    Serial.println(curr_angular_pos);
    Serial.print(", Ang Speed: ");
    Serial.println(curr_angular_speed);
    Serial.print(", Error: ");
    Serial.println(curr_error);
    Serial.print(", PID output: ");
    Serial.println(pid_output);
    Serial.print(", PWM output: ");
    Serial.println(pwm_output);
}

// encoder interrupt function, fires on change of either encoder input signal
void encoder_isr() {
    static uint8_t enc_val = 0; // static allows this value to persist across function calls

    enc_val = enc_val << 2; // shift the previous state to the left
    enc_val = enc_val | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits

    count += encoder_table[enc_val & 0b1111];    // preform the table lookup and increment count accordingly
}

void motorBackward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}
void motorForward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

