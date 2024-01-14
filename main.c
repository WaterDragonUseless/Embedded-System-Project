#include "stm32f10x.h"
#include "AD.h"

// Initialize PID control
void PID_Init(void) {
    AD_Init(); // Initialize AD converter as part of PID initialization
}

// PID control algorithm
float PID(uint16_t Aim, uint16_t D) {
    uint16_t Now;
    float Speed = 0;
    Now = AD_GetValue(ADC_Channel_8); // Get current ADC value

    // Calculate speed based on target and error margin
    if (Now < (Aim - D)) {
        Speed = ((Now - Aim) / 70) - 50;
        return Speed; // Return calculated speed
    } else if (Now > (Aim + D)) {
        Speed = ((Now - Aim) / 7) + 30;
        return Speed;
    } else {
        return 0; // No adjustment needed
    }
}

#include "stm32f10x.h"
#include "Delay.h"
#include "Motor.h"
#include "PID.h"
#include "OLED.h"
#include "AD.h"

// Main program entry point
int main(void) {
    float Speed;
    Motor_Init(); // Initialize motor
    PID_Init(); // Initialize PID control
    OLED_Init(); // Initialize OLED display

    // Main loop
    while (1) {
        Speed = PID(3500, 75); // Calculate speed using PID control
        Motor_L_SetSpeed(Speed); // Set left motor speed
        Motor_R_SetSpeed(Speed); // Set right motor speed
        OLED_ShowNum(1, 1, Speed + 100, 3); // Display speed on OLED
        OLED_ShowNum(2, 1, AD_GetValue(ADC_Channel_8), 4); // Display ADC value on OLED
    }
}
