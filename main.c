#include <stdint.h>
#include "tm4c123gh6pm.h"


#define STCTRL *((volatile uint32_t *) 0xE000E010)        // Control and Status Register
#define STRELOAD *((volatile uint32_t *) 0xE000E014)      // Reload Value Register
#define STCURRENT *((volatile uint32_t *) 0xE000E018)     // Current Value Register


#define ENABLE (1 << 0)                                   // Enable SysTick timer
#define CLKINT (1 << 2)                                   // Use system clock
#define CLOCK_HZ 16000000                                 // System clock frequency in Hz
#define SYSTICK_RELOAD(us) ((CLOCK_HZ / 1000000) * (us) - 1) // Calculates reload value for given microseconds

void I2C0_init(void) {
    SYSCTL_RCGCI2C_R |= 0x01;                             // Enable clock for I2C0 module
    SYSCTL_RCGCGPIO_R |= 0x02;                            // Enable clock for Port B

    while ((SYSCTL_PRGPIO_R & 0x02) == 0);                // Wait until Port B is ready

    GPIO_PORTB_AFSEL_R |= 0x0C;                           // Enable alternate function for PB2, PB3
    GPIO_PORTB_ODR_R |= 0x08;                             // Set PB3 (SDA) as open-drain
    GPIO_PORTB_DEN_R |= 0x0C;                             // Enable digital function on PB2, PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0xFF00) | 0x3300; // Assign I2C function to PB2, PB3

    I2C0_MCR_R = 0x10;                                    // Configure I2C0 as master
    I2C0_MTPR_R = 0x07;                                   // Set I2C clock frequency
}

void I2C0_send(uint8_t address, uint8_t msb, uint8_t lsb) {
    I2C0_MSA_R = (address << 1);                          // Set slave address and write mode
    I2C0_MDR_R = msb;                                     // Send the MSB
    I2C0_MCS_R = 0x03;                                    // Start and send
    while (I2C0_MCS_R & 0x01);                            // Wait for transfer to complete
    if (I2C0_MCS_R & 0x02) return;                        // If an error occurs, exit

    I2C0_MDR_R = lsb;                                     // Send the LSB
    I2C0_MCS_R = 0x05;                                    // Send and stop
    while (I2C0_MCS_R & 0x01);                            // Wait for transfer to complete
    if (I2C0_MCS_R & 0x02) return;                        // If an error occurs, exit
}



int main(void) {
    systick_config();                                     // Configure SysTick
    generate_sawtooth_samples();                          // Fill samples array with sawtooth data
    while (1) {
        generate_waveform();                              // Output waveform continuously
    }
}

