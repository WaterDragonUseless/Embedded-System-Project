#include "mbed.h"
#include "C12832.h"
#include "QEI.h"


class Encoder {
private:
    float previousPulses, RPM;
    QEI encoderQEI;

public:
    // Constructor initializes the QEI object and sets previousPulses to zero
    Encoder(PinName A, PinName B, PinName Index, int PPR) 
        : encoderQEI(A, B, Index, PPR), previousPulses(0.0f) {}

    // Update RPM value based on the change in pulses
    void updateRPM() {
        float currentPulses = encoderQEI.getPulses();
        RPM = (currentPulses - previousPulses) * 12.5f; // Update formula if necessary
        previousPulses = currentPulses;
    }

    // Get the current RPM
    float getRPM() const {
        return RPM;
    }

    // Get the current pulse count
    float getPulseCount() const {
        return encoderQEI.getPulses();
    }
};

class RPMcal : public Encoder {
private:
    Ticker rpmTicker;

public:
    // Constructor
    RPMcal(PinName A, PinName B, PinName Index, int PPR) 
        : Encoder(A, B, Index, PPR) {
        // Attach the updateRPM method to the ticker
        rpmTicker.attach(callback(this, &RPMcal::updateRPM), 0.1f);
    }
};



/*****Objects Declaration*****/

RPMcal l_speed( PC_9, PB_8 , NC, 256);
RPMcal r_speed(PB_3, PA_10 , NC, 256);
DigitalOut sensor0 (PC_12);
DigitalOut sensor1 (PC_10);
DigitalOut sensor2 (PB_9);
DigitalOut sensor3 (PA_3);
DigitalOut sensor4 (PC_6);
AnalogIn R0 (PC_2);
AnalogIn R1 (PC_3);
AnalogIn R2 (PC_5);
AnalogIn R3 (PB_1);
AnalogIn R4 (PC_4);
Serial hm10(PA_11, PA_12);
PwmOut MOTOR_L(PB_13);
PwmOut MOTOR_R(PB_14);
DigitalOut bipolar_1 (PC_8); //
DigitalOut bipolar_2 (PA_13);
DigitalOut enable (PB_2);

Ticker myticker;
float volatile initial_speed_l = 0.38, initial_speed_r = 0.37;
int volatile i = 0;
int volatile j = 0;


// Constants for motor speeds and timings
const float TURNAROUND_SPEED = 0.5;
const float TURNAROUND_TIME_1 = 0.4;
const float TURNAROUND_SPEED_ADJUST_L = 0.66;
const float TURNAROUND_SPEED_ADJUST_R = 0.34;
const float TURNAROUND_TIME_2 = 1.05;
const float NORMAL_SPEED_L = 0.38;
const float NORMAL_SPEED_R = 0.37;
const float TURNAROUND_TIME_3 = 0.1;

void turnaround() {
    MOTOR_L.write(TURNAROUND_SPEED);
    MOTOR_R.write(TURNAROUND_SPEED);
    wait(TURNAROUND_TIME_1);

    MOTOR_L.write(TURNAROUND_SPEED_ADJUST_L);
    MOTOR_R.write(TURNAROUND_SPEED_ADJUST_R);
    wait(TURNAROUND_TIME_2);

    MOTOR_L.write(NORMAL_SPEED_L);
    MOTOR_R.write(NORMAL_SPEED_R);
    wait(TURNAROUND_TIME_3);
}

void leftControl(float speed, float desiredSpeed) {
    const float Kp = 0.0001; // Proportional gain for the control
    float dutyCycle = 0.0; // Initialize the duty cycle

    // Calculate the difference between desired speed and actual speed
    float delta = desiredSpeed - speed;

    // Adjust the duty cycle based on the error (delta)
    dutyCycle -= Kp * delta;

    // Ensure dutyCycle is within the valid range (0.0 to 1.0)
    dutyCycle = std::max(0.0f, std::min(dutyCycle, 1.0f));

    MOTOR_L.write(dutyCycle);
}


void adjustMotorSpeedsBasedOnAverage(float avg, float& initial_speed_l, float& initial_speed_r, int& A) {
    const float LOW_SPEED_THRESHOLD = 10.0f;
    const float HIGH_SPEED_THRESHOLD = 110.0f;
    const float LOW_SPEED = 0.3f;
    const float MEDIUM_SPEED_L = 0.38f;
    const float MEDIUM_SPEED_R = 0.37f;
    const float HIGH_SPEED = 0.46f;

    if (avg < LOW_SPEED_THRESHOLD) {
        initial_speed_l = LOW_SPEED;
        initial_speed_r = LOW_SPEED;
        A = 1;
    } else if (avg > HIGH_SPEED_THRESHOLD) {
        initial_speed_l = HIGH_SPEED;
        initial_speed_r = HIGH_SPEED;
        A = 3;
    } else {
        initial_speed_l = MEDIUM_SPEED_L;
        initial_speed_r = MEDIUM_SPEED_R;
        A = 0;
    }

const float THRESHOLD_HIGH = 2.85f;
const float THRESHOLD_MIDDLE = 2.5f;
const float THRESHOLD_LOW = 2.3f;

int calculateError(float initial) {
    float R0_value = R0.read() * 3;
    float R1_value = R1.read() * 3;
    float R2_value = R2.read() * 3;
    float R3_value = R3.read() * 3;
    float R4_value = R4.read() * 3;

    // Check various sensor conditions and return corresponding error values
    if (R0_value > THRESHOLD_HIGH && R1_value > initial && R2_value > initial && R3_value > initial && R4_value > initial)
        return 10;
    
    if (R0_value > initial && R1_value > THRESHOLD_MIDDLE && R2_value < 2.55f && R3_value < THRESHOLD_LOW && R4_value > initial)
        return 1;

    if (R0_value > 2.6f && R1_value > THRESHOLD_MIDDLE && R2_value > 2.55f && R3_value < 2.4f && R4_value > 2.5f)
        return 1;

    if (R0_value > 2.6f && R1_value > THRESHOLD_MIDDLE && R2_value > 2.55f && R3_value < THRESHOLD_MIDDLE && R4_value < THRESHOLD_MIDDLE)
        return 2;

    // ... continue with other conditions ...

    if (R0_value < THRESHOLD_HIGH && R1_value < 2.53f && R2_value > initial && R3_value > initial && R4_value > initial)
        return -2;

    if (R0_value > initial && R1_value > 2.52f && R2_value < initial && R3_value > initial && R4_value > initial)
        return 0;

    // Default case if none of the above conditions are met
    return 0;
}


// Function to adjust motor speeds
void adjustMotorSpeeds(float leftAdjust, float rightAdjust) {
    MOTOR_L.write(initial_speed_l + leftAdjust);
    MOTOR_R.write(initial_speed_r + rightAdjust);
}

// Switch statement for error handling
switch(error) {
    case 10:
        MOTOR_L.write(0.5);
        MOTOR_R.write(0.5);
        break;
    case -1:
        adjustMotorSpeeds(SPEED_ADJUSTMENT_SMALL, -SPEED_ADJUSTMENT_SMALL);
        break;
    case -2:
        adjustMotorSpeeds(SPEED_ADJUSTMENT_MEDIUM, -SPEED_ADJUSTMENT_MEDIUM);
        break;
    case -4:
        adjustMotorSpeeds(SPEED_ADJUSTMENT_LARGE, -SPEED_ADJUSTMENT_LARGE);
        break;
    case 1:
        adjustMotorSpeeds(-SPEED_ADJUSTMENT_SMALL, SPEED_ADJUSTMENT_SMALL);
        break;
    case 2:
        adjustMotorSpeeds(-SPEED_ADJUSTMENT_MEDIUM, SPEED_ADJUSTMENT_MEDIUM);
        break;
    case 4:
        adjustMotorSpeeds(-SPEED_ADJUSTMENT_LARGE, SPEED_ADJUSTMENT_LARGE);
        break;
    case 0:
        adjustMotorSpeeds(0, 0);
        break;
    // Consider adding a default case for unexpected error values
    default:
        // Handle unexpected error value
        break;
}


int main() {
    hm10.baud(9600);
   
    float cure;
    int A = 0;
    int error;
    float initial = 2.6;
    float middle = 2.6;
    float avg ;
    sensor0 = 1;
    sensor2 = 1;
    sensor3 = 1;
    sensor4 = 1 ;
    sensor1 = 1;
    enable = 0;
    MOTOR_R.period(0.00005f);
    MOTOR_L.period(0.00005f);
    bipolar_1 = 1;
    bipolar_2 = 1;
    wait(0.5);
    cure = 0.025;
    error = 0;
    wait(2);
    float basic = 0.33;

    while(1){
    enable = 1;
   


     avg = l_speed.getRPM().05 + r_speed.getRPM().05;
 if(avg <10.0f)
 {
     initial_speed_l= 0.3f;
     initial_speed_r = 0.3f;
     A=1;
     }else if(avg >110.0f)
     {
         initial_speed_l= 0.46f;
         initial_speed_r = 0.46f;
         A = 3;
         }else
     {
        initial_speed_l= 0.38f;
        initial_speed_r = 0.37f;
        A = 0;
     }
    int error = calculateError(initial);
    adjustMotorSpeedsBasedOnAverage(avg, initial_speed_l, initial_speed_r, A);
    float adjustmentForLeftMotor = 0.05;  // Example adjustment values
    float adjustmentForRightMotor = -0.03;

    adjustMotorSpeeds(adjustmentForLeftMotor, adjustmentForRightMotor);
    
    /***deslpaying the readings*/
     if(hm10.readable())
     {
         c = hm10.getc();
         if(c == 'A'){
                 turnaround();
         }
         }
   
    lcd.locate(20,0);
    lcd.printf("error = %d \n AVG =%.2f \n A = %d",error, avg, A);
    }
 
   
   
};