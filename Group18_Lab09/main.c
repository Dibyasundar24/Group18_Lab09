#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

// Define DAC characteristics
#define MAX_DAC_VALUE 4095   // 12-bit DAC max value for sawtooth waveform

// Global variable to store the current DAC value
volatile uint16_t dacValue = 0;

void SPI_Init(void);
void Timer0A_Init(uint32_t period);
void Timer0A_Handler(void);

int main(void) {
    // Set system clock to 40 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize SPI for DAC communication and Timer0A
    SPI_Init();
    Timer0A_Init(SysCtlClockGet() / 1000);  // Set timer period to generate a 1 kHz waveform

    // Enable processor interrupts
    IntMasterEnable();

    while (1) {
        // Main loop can remain empty as the waveform generation is handled by the Timer0A interrupt
    }
}

// SPI Initialization for communicating with DAC
void SPI_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);       // Enable SSI0 (SPI0) module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);      // Enable GPIO Port A for SPI

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);               // Configure PA2 for SSI0 Clock
    GPIOPinConfigure(GPIO_PA4_SSI0RX);                // Configure PA4 for SSI0 RX (unused)
    GPIOPinConfigure(GPIO_PA5_SSI0TX);                // Configure PA5 for SSI0 TX
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);               // Configure PA3 for SSI0 FSS (Chip Select)

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 16);  // 1 MHz, 16-bit data

    SSIEnable(SSI0_BASE);
}

// Timer0A Initialization for periodic interrupts
void Timer0A_Init(uint32_t period) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);      // Enable Timer0 peripheral
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);   // Configure Timer0 as periodic
    TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);    // Set timer period

    IntEnable(INT_TIMER0A);                            // Enable Timer0A interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);   // Enable timeout interrupt
    TimerEnable(TIMER0_BASE, TIMER_A);                 // Start Timer0A
}

// Timer0A interrupt handler
void Timer0A_Handler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);    // Clear the timer interrupt

    // Increment DAC value to generate a sawtooth waveform
    dacValue += 50;                                    // Increase value for sawtooth effect
    if (dacValue > MAX_DAC_VALUE) {
        dacValue = 0;                                  // Reset value for sawtooth
    }

    // Format DAC data for MCP4921 (example DAC, 12-bit, write command format: 0x3 << 12)
    uint16_t dacCommand = (0x3 << 12) | (dacValue & 0x0FFF);

    // Send data to DAC over SPI
    while (SSIBusy(SSI0_BASE));                        // Wait if SSI is busy
    SSIDataPut(SSI0_BASE, dacCommand);                 // Send DAC command and data
}
