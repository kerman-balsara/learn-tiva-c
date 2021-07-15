// Kitchen oven timer implementation
// DISPLAY_CLOCK:
// - Shows the current time (hard-coded)
// - The clock is updated every minute
// - SW1 is used to change to DISPLAY_ALARM_INIT state
// - SW2 is used to change to DISPLAY_ALARM state
// DISPLAY_ALARM_INIT
// - Shows alarm time as  0:00
// - We can be in this state for 10 seconds only; after 10 seconds we go to DISPLAY_CLOCK state
// - SW1 remains in the same state (the ten second timer restarts)  
// - SW2 is used to change to DISPLAY_ALARM state and increment the alarm time
// DISPLAY_ALARM
// - Shows the alarm time as a countdown
// - SW1 is used to decrement the alarm time; if 0:00 we change to DISPLAY_ALARM_INIT state; SW2 is used to increment the alarm time;
//   in both cases, the alarm timer restarts
// - The alarm time is updated every minute
// When the alarm goes off:
// - The red led comes on
// - We go back to DISPLAY_CLOCK state
// - The alarm can be on for up to 10 seconds or until SW1 is pressed; in both cases the red led is switched off.

// We are using the LM4F, Breadboard (B), passive buzzer (PBZR), servo motor SG90, and the 5V supply from the UNO (the servo requires 5V).
// LM4F Connections: GND to B(GND), PF2 to PBZR +, PC4 to SG90 signal, USB to PC
// PBZR Connections: This component is not polarised. + to LM4F PF2, - to B(GND).
// SG90 Connections: + to B(+), - to B(GND), signal to LM4F PC4
// UNO Connections: +5V to B(+), GND to B(GND), USB to PC.

// LM4F components used: Systick, UART0, PF0 (Switch 2), PF1 (Red led), PF2 (PBZR), PF4 (Switch 1), PC4 (SG90), TIMER1 (PBZR), WTIMER0 (SG90) 

// Servo added just for fun.

// In system_TM4C123.c, CLOCK_SETUP = 0; we are using 16MHz clock

#include "stdio.h"
#include "stdlib.h"

#include "TM4C123GH6PM.h"
#include "gpiortns.h"
#include "uartrtns.h"
#include "driverlib/sysctl.h"

// Current time
#define CURRENT_HH 17
#define CURRENT_MM 44

// Cater for switch bounce
#define DEBOUNCE_TIME 200   // milliseconds

// TIMER1 setup for passive buzzer. The buzzer can operate at various frequencies. We have picked 400Hz (least annoying sound).
// Our clock frequency is 16MHz. Our required frequency is 400Hz.
// Load Value = ((1/400) * 16000000) = 40000
// Frequency    LOAD value
// 200Hz        80000 ((1/200) * 16000000)
// 400Hz        40000
// 800Hz        20000
// The match value controls the volume. If the match value is lowered, the buzzer is not heard.
#define TIMER1_LOAD  40000
#define TIMER1_MATCH 39900

// WTIMER0 setup for servo. The servo operates at 50Hz. It requires a pulse width of 0.6 to 2.4 milliseconds for the 180 degrees of rotation. 
// Our clock frequency is 16MHz. Our required frequency is 50Hz.
// Load Value = ((1/50) * 16000000) = 320000
// The time for the clock to count 1 is 1/16000000 = 0.0000000625 seconds (or 62.5ns)
// Hence, for a time period of 0.6ms, the clock would need to count 0.0006/0.0000000625 = 9600. The match value is 320000 - 9600 = 310400.
// Similarly, for a time period of 2.4ms, the clock would need to count 0.0024/0.0000000625 = 38400. The match value is 320000 - 38400 = 281600.
#define WTIMER0_LOAD  320000
#define WTIMER0_MATCH_M 296000 // 1.5ms
#define WTIMER0_MATCH_R 281600 // 2.4ms
#define WTIMER0_MATCH_L 310400 // 0.6ms
// Note: Based on the datasheet for SG90, the values between 1ms and 2ms provide only a 90 degree rotation.
//#define WTIMER0_MATCH_M 296000 // 1.5ms
//#define WTIMER0_MATCH_R 288000 // 2ms
//#define WTIMER0_MATCH_L 304000 // 1ms

// Delays
#define DELAY_TIME_1  1000        //  1 second
#define DELAY_TIME_3  3000        //  3 seconds
#define DELAY_TIME_10 10000       // 10 seconds
#define DELAY_TIME_15 15000       // 15 seconds
#define DELAY_TIME_60 60000       // 60 seconds

// States
#define DISPLAY_CLOCK       0
#define DISPLAY_ALARM_INIT  1
#define DISPLAY_ALARM       2

static volatile int32_t CurrentTicks;

// Queue switch inputs
#define MESSAGE_QUEUE_SZ 50

typedef struct
{
    int32_t switchPressed;
    int32_t switchPressedTime; // for debouncing
} msgque_t;

static msgque_t MessageQueue[MESSAGE_QUEUE_SZ];
static volatile int32_t FrontQueue = -1;
static volatile int32_t RearQueue = -1;

// Uart
static volatile uint16_t UartActionReqd;
static volatile char Uart0Char;

// Internal function prototypes
static void setup_uart0(void);
static void setup_leds(void);
static void setup_switch1(void);
static void setup_switch2(void);
static void setup_systick(void);
static void setup_pwm_buzzer(void);
static void setup_pwm_servo(void);

static void incrementTime(uint32_t * const hh,
                          uint32_t * const mm);
static void decrementTime(uint32_t * const hh,
                          uint32_t * const mm);
static void printTime(const uint32_t hh,
                      const uint32_t mm);

static void printChar(const char c);
static void printString(const char * string);
static char readChar(void);

static void enqueue(const int32_t input);
static void dequeue(int32_t * const switchPressed,
                    int32_t * const switchPressedTime);

// External function prototypes
void SysTick_Handler(void);
void GPIOF_Handler(void);
void UART0_Handler(void);

int main(void)
{
    int32_t previousClockTicks;
    int32_t previousAlarmTicks;
    int32_t previousSW1Ticks;
    int32_t ledOnTicks;
    int32_t lastLedOnTicks;

    int32_t switchPressedTime;
    
    int32_t lastSwitch1Processed;
    int32_t lastSwitch2Processed;
    
    
    setup_uart0();

    GpioEnable(PORT_F);
    setup_leds();
    setup_switch1();
    setup_switch2();
    
    setup_pwm_buzzer();
    setup_pwm_servo();

    // Reset servo
    WTIMER0->CTL = 0x0;
    WTIMER0->TAILR = WTIMER0_LOAD - 1;
    WTIMER0->TAMATCHR = WTIMER0_MATCH_M - 1 ;
    WTIMER0->CTL = 0x1;
    
    uint32_t pos = 0;

//    To call SysCtlClockGet():
//    - Change driverlib/sysctl.h to include <inc/hw_types.h>
//    - Change the calling program to include "driverlib/sysctl.h"
//    - Add driverlib-cm4f.lib to the project (to prevent linker error due to missing SysCtlClockGet())
//    volatile uint32_t clock_speed = SysCtlClockGet();
    char clockString[20 + 1];
    sprintf(clockString, "\n\r%lu\n\r", SysCtlClockGet());
    printString(clockString);
   
    uint32_t displayState = DISPLAY_CLOCK;
    uint32_t clockHH = CURRENT_HH;
    uint32_t clockMM = CURRENT_MM;
    
    printTime(clockHH, clockMM);

    uint32_t alarmHH = 0;
    uint32_t alarmMM = 0;
    
    // This must be the last step (unless some prior code requires CurrentTicks). 
    setup_systick();
    
    previousClockTicks = CurrentTicks;
    lastSwitch1Processed = lastSwitch2Processed = CurrentTicks;
    
    while (1)
    {
        // Buffer inputs; we want to use the same value throughout the loop.
        int32_t now = CurrentTicks;
        int32_t switchPressed;
        dequeue(&switchPressed, &switchPressedTime);
        
        // Cater for switch bounce
        if (switchPressed == 1)
        {
            if (abs(switchPressedTime - lastSwitch1Processed) < DEBOUNCE_TIME)
            {
                switchPressed = -1;
            }
            else
            {
                lastSwitch1Processed = switchPressedTime;
            }
        }
        else if (switchPressed == 2)
        {
            if (abs(switchPressedTime - lastSwitch2Processed) < DEBOUNCE_TIME)
            {
                switchPressed = -1;
            }
            else
            {
                lastSwitch2Processed = switchPressedTime;
            }
        }

        if (switchPressed == 2)
        {
            GPIOF->DATA &= ~(1U << 1);       // Turn red led off
            TIMER1->CTL = 0x00;

            displayState = DISPLAY_ALARM;
            incrementTime(&alarmHH, &alarmMM);
            printTime(alarmHH, alarmMM);

            previousAlarmTicks = now;
        }
        else if (switchPressed == 1)
        {
            if (GPIOF->DATA & (1 << 1))
            {
                GPIOF->DATA &= ~(1U << 1);       // Turn red led off
                TIMER1->CTL = 0x00;
            }
            else
            {
                if (alarmHH == 0 && alarmMM == 0)
                {
                    displayState = DISPLAY_ALARM_INIT;
                    previousSW1Ticks = now;
                }
                else if (displayState == DISPLAY_ALARM)
                {
                    decrementTime(&alarmHH, &alarmMM);
                    
                    if (alarmHH == 0 && alarmMM == 0)
                    {
                        displayState = DISPLAY_ALARM_INIT;
                        previousSW1Ticks = now;
                    }
                    else
                    {
                        previousAlarmTicks = now;
                    }
                }
                
                printTime(alarmHH, alarmMM);
            }
        }
        else // nothing in MessageQueue
        {
            if (displayState == DISPLAY_ALARM_INIT && abs(now - previousSW1Ticks) >= DELAY_TIME_10)
            {
                displayState = DISPLAY_CLOCK;
                printTime(clockHH, clockMM);
            }

            if (displayState == DISPLAY_ALARM && abs(now - previousAlarmTicks) >= DELAY_TIME_60)
            {
                decrementTime(&alarmHH, &alarmMM);
                if (alarmHH == 0 && alarmMM == 0)
                {
                    displayState = DISPLAY_CLOCK;
                    GPIOF->DATA |= (1 << 1);        // Turn red led on
                    TIMER1->CTL = 0x01;

                    ledOnTicks = lastLedOnTicks = now;
                    printTime(clockHH, clockMM);
                }
                else
                {
                    previousAlarmTicks = now;
                    printTime(alarmHH, alarmMM);
                }
            }

            if (GPIOF->DATA & (1 << 1))
            {
                if (abs(now - ledOnTicks) >= DELAY_TIME_15)
                {
                    GPIOF->DATA &= ~(1U << 1);       // Turn red led off
                    TIMER1->CTL = 0x00;
                }
                else if (abs(now - lastLedOnTicks) >= DELAY_TIME_1) // Turn buzzer on/off every second
                {
                    TIMER1->CTL ^= 1;
                    lastLedOnTicks = now;
                }
            }
        }

        if (abs(now - previousClockTicks) >= DELAY_TIME_60)
        {
            previousClockTicks = now;
            incrementTime(&clockHH, &clockMM);

            if (displayState == DISPLAY_CLOCK)
                printTime(clockHH, clockMM);

            // Change position of servo 90 degress every minute.
            pos++;
            if (pos > 3) pos = 0;

            WTIMER0->CTL = 0x0;
            WTIMER0->TAILR = WTIMER0_LOAD - 1;
            
            uint32_t match = pos == 1 ? WTIMER0_MATCH_L : pos == 3 ? WTIMER0_MATCH_R : WTIMER0_MATCH_M;
            WTIMER0->TAMATCHR = match - 1 ;
            WTIMER0->CTL = 0x1;
        }   
    }

    // Commented to prevent compiler warning
    // return(0);
}

void SysTick_Handler(void)
{
    CurrentTicks++;
}

// Handle SW1/SW2 pressed
void GPIOF_Handler(void)
{
    if (GPIOF->MIS & 0x10)      // SW1 pressed.
    {
        GPIOF->ICR |= (1 << 4); // Clear the interrupt
        enqueue(1);
    }
    else if (GPIOF->MIS & 0x01) // SW2 pressed.
    {   
        GPIOF->ICR |= (1 << 0); // Clear the interrupt
        enqueue(2);
    }
}

static void enqueue(const int32_t switchPressed)
{
    if (RearQueue == MESSAGE_QUEUE_SZ - 1)
    {
    }
    else
    {
        if (FrontQueue == -1 && RearQueue == -1)
        {
            FrontQueue = RearQueue = 0;
        }
        else
        {
            RearQueue++;
        }
        MessageQueue[RearQueue].switchPressed = switchPressed;
        MessageQueue[RearQueue].switchPressedTime = CurrentTicks;
    }
}    

static void dequeue(int32_t * const switchPressed,
                    int32_t * const switchPressedTime)
{
    *switchPressed = -1;
    if (FrontQueue == -1 && RearQueue == -1)
        return;
    
    *switchPressed = MessageQueue[FrontQueue].switchPressed;
    *switchPressedTime = MessageQueue[FrontQueue].switchPressedTime;
    
    MessageQueue[FrontQueue].switchPressed = -1;
    if (FrontQueue == RearQueue)
        FrontQueue = RearQueue = -1;
    else
        FrontQueue++;
}    

static void setup_leds(void)
{
    GPIOF->DIR |= (1 << 1);       // Set PF1 as digital output to control red LED
    GPIOF->DEN |= (1 << 1);       // Set PF1 as digital pin
}

static void setup_switch1(void)
{
    // Initialize PF4 as digital input pin
    GPIOF->DIR &= ~(1U << 4);      // Set PF4 as a digital input pin
    GPIOF->DEN |= (1 << 4);       // Set PF4 as digital pin
    GPIOF->PUR |= (1 << 4);       // Enable pull-up for PF4
    
    // Configure PF4 for falling edge trigger interrupt
    GPIOF->IS  &= ~(1U << 4);      // make bit 4 edge sensitive
    GPIOF->IBE &= ~(1U << 4);      // trigger is controlled by IEV
    GPIOF->IEV &= ~(1U << 4);      // falling edge trigger
    GPIOF->ICR |= (1 << 4);       // clear any prior interrupt
    GPIOF->IM  |= (1 << 4);       // unmask interrupt
    
    // Configure interrupt.
    // Set Interrupt Clear Register (Receive Interrupt Clear)
    // Set Interrupt Mask Register
    // Set Interrupt Set Enable Register through NVIC i.e. the interrupt must be enabled in the NVIC_ENm_R register.
    // Each NVIC_ENm_R register has 32-bits, and each bit controls one interrupt number. Using the following formula to find out NVIC_ENm_R register number
    // and the bit number to enable the interrupt on NVIC.
    // m = interrupt number / 32
    // b = interrupt number % 32
    // NVIC->ISER[m] |= (1UL << b);
    // For GPIOF, the interrupt number is 30 (see startup_TM4C123.s; look for GPIOF_Handler; you will find "30: GPIO Port F")
    // m = 30 / 32 = 0; hence we update ISER[0]; b = 30 % 32 = 30; hence we set bit 30.
    NVIC->IP[30] = 3 << 5;      // Set interrupt priority to 3
    NVIC->ISER[0] |= (1 << 30); // Enable IRQ30
}

static void setup_switch2(void)
{
	// PF0 has special function, need to unlock to modify
    GPIOF->LOCK = 0x4C4F434B;   // Unlock commit register
    GPIOF->CR = 0x01;           // Make PF0 configurable
    GPIOF->LOCK = 0;            // Lock commit register

    // Initialize PF0 as digital input pin
    GPIOF->DIR &= ~(1U << 0);  // Set PF0 as a digital input pin
    GPIOF->DEN |= (1 << 0);   // Set PF0 as digital pin
    GPIOF->PUR |= (1 << 0);   // Enable pull-up for PF0
    
    // Configure PF0 for falling edge trigger interrupt
    GPIOF->IS  &= ~(1U << 0);        // make bit 0 edge sensitive
    GPIOF->IBE &= ~(1U << 0);        // trigger is controlled by IEV
    GPIOF->IEV &= ~(1U << 0);        // falling edge trigger
    GPIOF->ICR |= (1 << 0);         // clear any prior interrupt
    GPIOF->IM  |= (1 << 0);         // unmask interrupt
    
    // Configure interrupt.
    // Set Interrupt Clear Register (Receive Interrupt Clear)
    // Set Interrupt Mask Register
    // Set Interrupt Set Enable Register through NVIC i.e. the interrupt must be enabled in the NVIC_ENm_R register.
    // Each NVIC_ENm_R register has 32-bits, and each bit controls one interrupt number. Using the following formula to find out NVIC_ENm_R register number
    // and the bit number to enable the interrupt on NVIC.
    // m = interrupt number / 32
    // b = interrupt number % 32
    // NVIC->ISER[m] |= (1UL << b);
    // For GPIOF, the interrupt number is 30 (see startup_TM4C123.s; look for GPIOF_Handler; you will find "30: GPIO Port F")
    // m = 30 / 32 = 0; hence we update ISER[0]; b = 30 % 32 = 30; hence we set bit 30.
    NVIC->IP[30] = 3 << 5;     // Set interrupt priority to 3
    NVIC->ISER[0] |= (1<<30);  // Enable IRQ30
}

static void setup_systick(void)
{
//Delay time calculation (assume 16MHz):
//Since we are working with external clock i.e. 16 MHz, each pulse generated by the clock source will have:
//1/XTAL frequency = 1/(16*10^6) = 62.5ns time period. 
//So if we load 253 into the RELOAD register and trigger the counter it will count down with next pulse and will take 62.5ns to change its value from 253 to 252.
//Hence, In order to generate the delay, we can calculate the approximate value that has to be loaded into this register by the formula-
//Reload Value = XTAL*Time delay
//one extra clock delay is already included to set the flag for rollover, hence we get one extra clock delay. By subtracting by one will give exact time delay.
//Reload Value = (XTAL*Time Delay)-1
//Remember that in one shoot, it can only take 0xFFFFFF maximum value (24 bits). Therefore in one shoot, we can only generate maximum of Time delay
//TimeDelay = (ReloadValue+1)/XTAL=16777215+(1/(16*10^6))
//TimeDelay = 1.048575937 sec.
//Example: For generating 1us (0.000001 second) delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*.000001)-1
//Reload Value = 15
//Example: For generating 1ms (0.001 second) delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*.001)-1
//Reload Value = 15999
//Example: For generating 1 second delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*1)-1
//Reload Value = 15999999
        
    // We will generate 1ms time delay using 16MHz (CLOCK_SETUP is 0)
    // Reload value = (16*10^6*.001)-1 = 15999
    // Configure clock. See Tiva C DS page 123 (System Timer). For configuring CTRL see page 138 (Register 1);
    // We need to set bits 0, 1, 2 i.e. 0111 == 07U or (1U << 2) | (1U << 1) | (1U)
    SysTick->CTRL = 0;
    SysTick->LOAD = 15999;  // Cannot exceed 24 bits i.e. 16777215
    SysTick->VAL  = 0U;
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

static void setup_uart0(void)
{
    //1. Enable the UART module using the RCGCUART register (see page 344)
    UartEnable(UART_0);
    //2. To find out which GPIO port to enable, refer to Table 23-5 on page 1351. UART0 uses port A (U0Rx PA0 Pin 17, U0Tx PA1 Pin 18)
    //   Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340).
    GpioEnable(PORT_A);
    //3. Set the GPIO AFSEL bits 0 and 1 (based on PA0 and PA1)for the appropriate pins (see page 671).
    GPIOA->AFSEL = (1 << 1) | (1 << 0);
    //4. Configure the GPIO current level and/or slew rate as specified for the mode selected (see page 673 and page 681).
    // Not required
    //5. Configure the PMCn fields in the GPIOPCTL register
    GPIOA->PCTL  = (1 << 0) | (1 << 4);
    GPIOA->DEN   = (1 << 0) | (1 << 1);

    // Configure UART0
    // The clock speed used to calculate IBRD/FBRD depends on the UARTCC Register setting of the UART Clock and if the PLL is used or not.
    // If the UARTCC shows PIOSC clock (0x05) as the clock source, then it always is 16MHz and the Baud rate needs to be computed off it (CLOCK_SETUP value has no effect).
    // IBRD=104, FBRD=11
    // If the UARTCC shows the System Clock (0x00) as the clock source, then the RCC-RCC2 registers need to be checked to determine the System Clock and then compute the Baud Rate.
    // CLOCK_SETUP = 1: Use 50MHz system clock, IBRD=325, FBRD=33
    // CLOCK_SETUP = 0: Use 16MHz system clock, IBRD=104, FBRD=11
    // In our code, we are using the 16MHz clock (UARTCC 0x00 with CLOCK_SETUP = 0) 
    UART0->CTL  &= ~(1U << 0);
    UART0->CC   = 0x0;
    UART0->IBRD = 104;
    UART0->FBRD = 11;
    UART0->LCRH = (0x3 << 5);
    UART0->CTL  = (1 << 0) | (1 << 8) | (1 << 9);

    // Configure interrupt.
    // Set Interrupt Clear Register (Receive Interrupt Clear)
    // Set Interrupt Mask Register
    // Set Interrupt Set Enable Register through NVIC i.e. the interrupt must be enabled in the NVIC_ENm_R register.
    // Each NVIC_ENm_R register has 32-bits, and each bit controls one interrupt number. Using the following formula to find out NVIC_ENm_R register number
    // and the bit number to enable the interrupt on NVIC.
    // m = interrupt number / 32
    // b = interrupt number % 32
    // NVIC->ISER[m] |= (1UL << b);
    // For UART0, the interrupt number is 5 (see startup_TM4C123.s; look for UART0_Handler; you will find "5: UART0 Rx and Tx")
    // m = 5 / 32 = 0; hence we update ISER[0]; b = 5 % 32 = 5; hence we set bit 5.
    UART0->ICR &= ~(1U << 4);
    UART0->IM |= (1 << 4);
    NVIC->ISER[0] = (1 << 5);
}

void UART0_Handler(void)
{
    Uart0Char = readChar();
    UartActionReqd = 1;
}

static char readChar(void)
{
    char c;
    while ((UART0->FR & (1 << 4)) != 0);
    c = UART0->DR;
    return c;
}

static void incrementTime(uint32_t * const hh,
                          uint32_t * const mm)
{
    *mm += 1;
    if (*mm == 60)
    {
        *hh += 1;
        if (*hh == 24) *hh = 0;
        *mm = 0;
    }
}

static void decrementTime(uint32_t * const hh,
                          uint32_t * const mm)
{
    if (*hh == 0 && *mm == 0)
        return;

    if (*mm == 0)
    {
        *hh -= 1;
        *mm = 59;
    }
    else
    {
        *mm -= 1;
    }
}

static void printTime(const uint32_t hh,
                      const uint32_t mm)
{
    char timeString[6 + 1];
    sprintf(timeString, "%2d:%02d\r", hh, mm);
    printString(timeString);
}

static void printString(const char * string)
{
    while (*string)
    {
        printChar(*(string++));
    }
}

static void printChar(const char c)
{
    while ((UART0->FR & (1 << 5)) != 0);
    UART0->DR = c;
}

// LM4F does not have a PWM module so we are using TIMER1A to generate a PWM signal for the buzzer. 
// We are using GPIO PF2 alternate function for PWM.
// Pin Name: T1CCP0, Pin Number: 30, Pin Mux/Pin Assignment: PF2 (7), Pin Type: I/O, Buffer Type: TTL, Description: 16/32-Bit Timer 1 Capture/Compare/PWM 0.
// See Table 11-2.
static void setup_pwm_buzzer(void)
{
    // Disable clock gating for TIMER module.
    SYSCTL->RCGCTIMER |= (1 << 1);  // TIMER1

    // There must be a delay of 3 system clocks after a peripheral module clock is enabled in the RCGC register
    // before any module registers are accessed. See page 227 (System Control). We check PRTIMER.
    while (1)
    {
        if (SYSCTL->PRTIMER & (1 << 1))
            break;
    }

    // Configure GPIO PF2. Use "Digital Output (Timer PWM)" from "Table 10-3. GPIO Pad Configuration Examples" as a reference.
    GpioEnable(PORT_F);
    GPIOF->AFSEL |= (1 << 2);
    GPIOF->PCTL &= ~0x00000700U;    // see Table 11-2 for value (7 for PF2/T1CCP0) and Register GPIOPCTL for bit position (11:8 PMC2 ...).
    GPIOF->PCTL |= 0x00000700;
    GPIOF->DEN |= (1 << 2);         // Set PF2 as digital pin

//In PWM Timing mode, the timer continues running after the PWM signal has been generated. The
//PWM period can be adjusted at any time by writing the GPTMTnILR register, and the change takes
//effect at the next cycle after the write
//1. Ensure the timer is disabled (the TnEN bit is cleared) before making any changes.
    TIMER1->CTL = 0x00;
    
//2. Write the GPTM Configuration (GPTMCFG) register with a value of 0x0000.0004.
    TIMER1->CFG = 0x04;             // 16-bit timer
    
//3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnAMS bit to 0x1, the TnCMR bit to
//0x0, and the TnMR field to 0x2.
    TIMER1->TAMR = 0x2;             // Periodic Timer mode
    TIMER1->TAMR |= (1 << 3);       // PWM is enabled

//4. Configure the output state of the PWM signal (whether or not it is inverted) in the TnPWML field
//of the GPTM Control (GPTMCTL) register.
    // Nothing to do
    
//5. If a prescaler is to be used, write the prescale value to the GPTM Timer n Prescale Register
//(GPTMTnPR).
    // Nothing to do
//6. If PWM interrupts are used, configure the interrupt condition in the TnEVENT field in the
//GPTMCTL register and enable the interrupts by setting the TnPWMIE bit in the GPTMTnMR
//register. Note that edge detect interrupt behavior is reversed when the PWM output is inverted
//(see page 696).
    // Nothing to do
    
//7. Load the timer start value into the GPTM Timer n Interval Load (GPTMTnILR) register.
    TIMER1->TAILR = TIMER1_LOAD - 1;
    
//8. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match value.
    TIMER1->TAMATCHR = TIMER1_MATCH - 1;

//9. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the timer and begin
//generation of the output PWM signal.
//    Done in main loop as and when required
}

// LM4F does not have a PWM module so we are using WTIMER0A to generate a PWM signal for the servo. We use a wide timer (32 bit) since the load value (WTIMER0_LOAD)
// does not fit in a normal 16-bit timer. 
// We are using GPIO PC4 alternate function for PWM.
// Pin Name: WT0CCP0, Pin Number: 16, Pin Mux/Pin Assignment: PC4 (7), Pin Type: I/O, Buffer Type: TTL, Description: 32/64-Bit Wide Timer 0 Capture/Compare/PWM 0.
// See Table 11-2.
static void setup_pwm_servo(void)
{
    // Disable clock gating for WTIMER module.
    SYSCTL->RCGCWTIMER |= (1 << 0);  // WTIMER0
    // There must be a delay of 3 system clocks after a peripheral module clock is enabled in the RCGC register
    // before any module registers are accessed. See page 227 (System Control). We check PRWTIMER.
    while (1)
    {
        if (SYSCTL->PRWTIMER & (1 << 0))
            break;
    }

    // Configure GPIO PC4. Use "Digital Output (Timer PWM)" from "Table 10-3. GPIO Pad Configuration Examples" as a reference.
    GpioEnable(PORT_C);
    GPIOC->AFSEL |= (1 << 4);
    GPIOC->PCTL &= ~0x00070000U;  // see Table 11-2 for value (7 for PC4/WT0CCP0) and Register GPIOPCTL for bit position (19:16 PMC4 ...)
    GPIOC->PCTL |= 0x00070000;
    GPIOC->DEN |= (1 << 4);       // Set PC4 as digital pin

//In PWM Timing mode, the timer continues running after the PWM signal has been generated. The
//PWM period can be adjusted at any time by writing the GPTMTnILR register, and the change takes
//effect at the next cycle after the write
//1. Ensure the timer is disabled (the TnEN bit is cleared) before making any changes.
    WTIMER0->CTL = 0x00;
    
//2. Write the GPTM Configuration (GPTMCFG) register with a value of 0x0000.0004.
    WTIMER0->CFG = 0x4;              // 32-bit timer
    
//3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnAMS bit to 0x1, the TnCMR bit to
//0x0, and the TnMR field to 0x2.
    WTIMER0->TAMR = 0x2;             // Periodic Timer mode
    WTIMER0->TAMR |= (1 << 3);       // PWM is enabled

//4. Configure the output state of the PWM signal (whether or not it is inverted) in the TnPWML field
//of the GPTM Control (GPTMCTL) register.
    // Nothing to do
    
//5. If a prescaler is to be used, write the prescale value to the GPTM Timer n Prescale Register
//(GPTMTnPR).
    // Nothing to do
//6. If PWM interrupts are used, configure the interrupt condition in the TnEVENT field in the
//GPTMCTL register and enable the interrupts by setting the TnPWMIE bit in the GPTMTnMR
//register. Note that edge detect interrupt behavior is reversed when the PWM output is inverted
//(see page 696).
    // Nothing to do
    
//7. Load the timer start value into the GPTM Timer n Interval Load (GPTMTnILR) register.
//   Done in main loop as and when required
    
//8. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match value.
//    Done in main loop as and when required

//9. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the timer and begin
//generation of the output PWM signal.
//    Done in main loop as and when required
}
