#include <Motor.h>

// pin assignment for the encoders
#define ENC_A (35)   // Encoder Input A for motor A
#define ENC_B (36)   // Encoder Input B for motor A
#define ENC_C (27)   // Encoder Input A for motor B
#define ENC_D (28)   // Encoder Input A for motor B

// global variables
volatile int32_t count_A = 0;   // encoder step count, 32 bit volatile integer (volatile ensures the value is fetched everytime, necessary for global variables modified by interrupt handlers)
volatile int32_t count_B = 0;
const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Instantiate 2 Motor objects
Motor motorA(0,22,35,36,18,19);
Motor motorB(1,2,27,28,25,26);

void setup() {
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_C, INPUT);
  pinMode(ENC_D, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderA_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderA_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C), encoderB_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D), encoderB_isr, CHANGE);
}

void loop() {
  delay(150);
  motorA.setNewSpeed(7);
  motorB.setNewSpeed(7);
  
  noInterrupts();
  motorA.getCount(count_A, count_B);
  interrupts();
  motorA.pidControl();

  noInterrupts();
  motorB.getCount(count_A, count_B);
  interrupts();
  motorB.pidControl();
}


// motor A's encoder ISR, fires on change of either encoder input signal
void encoderA_isr() {
    static uint8_t enc_val_A = 0; // static allows this value to persist across function calls

    enc_val_A = enc_val_A << 2; // shift the previous state to the left
    enc_val_A = enc_val_A | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits

    count_A += encoder_table[enc_val_A & 0b1111];    // preform the table lookup and increment count_A accordingly
}

// motor B's encoder ISR
void encoderB_isr() {
    static uint8_t enc_val_B = 0; // static allows this value to persist across function calls

    enc_val_B = enc_val_B << 2; // shift the previous state to the left
    enc_val_B = enc_val_B | ((PIND & 0b1100) >> 2); // or the current state into the 2 rightmost bits

    count_B += encoder_table[enc_val_B & 0b1111];    // preform the table lookup and increment count_A accordingly
}


