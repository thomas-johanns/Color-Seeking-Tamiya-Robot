#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "uart.h"
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include <string.h>
#include <avr/eeprom.h>
#include <math.h>

#define TCS_ADDR         0x29
#define BNO_ADDR         0x28
#define BTN1             (!(VPORTB.IN & PIN2_bm))
#define PWM_PERIOD_TICKS 16000
#define DRIVE_TIME       6026

// pitch 18 is arms all the way down for driving and scanning
// pitch -6 is arms up carrying a block
#define PITCH_DRIVE  18.0f
#define PITCH_CARRY  -6.0f

volatile int timer_count;

// sets the clock to use the external 16mhz crystal
void init_clock(void)
{
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.XOSCHFCTRLA = CLKCTRL_FRQRANGE_16M_gc | CLKCTRL_ENABLE_bm;
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_EXTCLK_gc;
    while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm));
}

// timer that fires every 2ms and increments timer_count
// used for all timing in the robot
void InitTCA1(void)
{
    TCA1.SINGLE.CTRLB   = TCA_SINGLE_WGMODE_NORMAL_gc;
    TCA1.SINGLE.PER     = 499;
    TCA1.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA1.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
}

ISR(TCA1_OVF_vect)
{
    timer_count += 2;
    TCA1.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm;
}

// adc setup not really used but kept from original code
void InitADC(void)
{
    ADC0.MUXPOS  = ADC_MUXPOS_AIN21_gc;
    ADC0.CTRLC   = ADC_PRESC_DIV8_gc;
    ADC0.CTRLD   = ADC_INITDLY_DLY16_gc;
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;
    ADC0.CTRLA   = ADC_ENABLE_bm;
}

// pwm on pc2 at 50 percent duty cycle powers the motors through l298n
void InitMotorPWM(void)
{
    TCA0.SINGLE.CTRLB  = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER    = PWM_PERIOD_TICKS - 1;
    TCA0.SINGLE.CMP2   = PWM_PERIOD_TICKS / 2 - 1;
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
    PORTC.DIRSET = PIN2_bm;
}

// drive motors on portd 0-3 and arm motor on porte 0-1
void InitMotorPins(void)
{
    PORTD.DIRSET = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;
    PORTD.OUTCLR = PIN0_bm|PIN1_bm|PIN2_bm|PIN3_bm;
    PORTE.DIRSET = PIN0_bm|PIN1_bm;
    PORTE.OUTCLR = PIN0_bm|PIN1_bm;
}

// sets up the tcs34725 color sensor with gain and integration time
void tcs_init(void)
{
    TWI_Address(TCS_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x80);
    TWI_Transmit_Data(0x01);
    TWI_Stop();
    _delay_ms(3);
    TWI_Address(TCS_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x80);
    TWI_Transmit_Data(0x13);
    TWI_Stop();
    TWI_Address(TCS_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x81);
    TWI_Transmit_Data(0xC0);
    TWI_Stop();
    TWI_Address(TCS_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x8F);
    TWI_Transmit_Data(0x00);
    TWI_Stop();
}

// reads one channel from the color sensor at the given register
uint16_t tcs_read_channel(uint8_t reg)
{
    TWI_Address(TCS_ADDR, TW_WRITE);
    TWI_Transmit_Data(0xA0 | reg);
    TWI_Stop();
    uint8_t low  = TWI_Host_Read(TCS_ADDR);
    uint8_t high = TWI_Host_Read(TCS_ADDR);
    return (uint16_t)(high << 8) | low;
}

// reads all four channels from the color sensor red green blue and clear
void tcs_read_rgbc(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    *c = tcs_read_channel(0x14);
    *r = tcs_read_channel(0x16);
    *g = tcs_read_channel(0x18);
    *b = tcs_read_channel(0x1A);
}

// motor a is the left drive motor
void motorA_forward(void)
{
    PORTD.OUTCLR = PIN0_bm;
    PORTD.OUTSET = PIN1_bm;
}

void motorA_reverse(void)
{
    PORTD.OUTSET = PIN0_bm;
    PORTD.OUTCLR = PIN1_bm;
}

void motorA_stop(void)
{
    PORTD.OUTCLR = PIN0_bm|PIN1_bm;
}

// motor b is the right drive motor
void motorB_forward(void)
{
    PORTD.OUTCLR = PIN2_bm;
    PORTD.OUTSET = PIN3_bm;
}

void motorB_reverse(void)
{
    PORTD.OUTSET = PIN2_bm;
    PORTD.OUTCLR = PIN3_bm;
}

void motorB_stop(void)
{
    PORTD.OUTCLR = PIN2_bm|PIN3_bm;
}

// arm motor raises and lowers the front arms
void arm_forward(void)
{
    PORTE.OUTCLR = PIN0_bm;
    PORTE.OUTSET = PIN1_bm;
}

void arm_reverse(void)
{
    PORTE.OUTSET = PIN0_bm;
    PORTE.OUTCLR = PIN1_bm;
}

void arm_stop(void)
{
    PORTE.OUTCLR = PIN0_bm|PIN1_bm;
}

// stops both drive motors
void robot_stop(void)
{
    motorA_stop();
    motorB_stop();
}

// reads the compass heading from the bno055 in degrees 0 to 360
float read_heading(void)
{
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x1A);
    TWI_Stop();
    uint8_t lo = TWI_Host_Read(BNO_ADDR);
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x1B);
    TWI_Stop();
    uint8_t hi = TWI_Host_Read(BNO_ADDR);
    return (float)(int16_t)((hi<<8)|lo) / 16.0f;
}

// reads the pitch angle from the bno055
// pitch 18 means arms are pointing down
// pitch -6 means arms are raised up
float read_pitch(void)
{
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x1E);
    TWI_Stop();
    uint8_t lo = TWI_Host_Read(BNO_ADDR);
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x1F);
    TWI_Stop();
    uint8_t hi = TWI_Host_Read(BNO_ADDR);
    return (float)(int16_t)((hi<<8)|lo) / 16.0f;
}

// returns the shortest angle difference between two headings
// result is between -180 and 180
float heading_diff(float target, float current)
{
    float diff = target - current;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// spins clockwise until the bno055 heading matches the target
// never switches direction so it cant twitch
void turn_to_heading(float target)
{
    LCDclr();
    LCDgotoXY(0, 0);
    char buf[16];
    sprintf(buf, "Turn->%.0f", target);
    LCDstring(buf);

    robot_stop();
    _delay_ms(300);

    motorA_forward();
    motorB_reverse();

    while (1)
    {
        float diff = heading_diff(target, read_heading());
        if (diff > -8.0f && diff < 8.0f)
        {
            robot_stop();
            break;
        }
    }
    _delay_ms(200);
}

// drives both motors forward for a set amount of milliseconds
void drive_forward(int duration_ms)
{
    int start = timer_count;
    motorA_forward();
    motorB_forward();
    while ((timer_count - start) < duration_ms);
    robot_stop();
}

// moves the arms until the pitch sensor reads the target angle
// has a timeout so it never gets stuck forever
void move_arms_to_pitch(float target)
{
    uint32_t timeout = 0;
    float p = read_pitch();
    if (p > target + 2.0f)
    {
        arm_reverse();
        while (read_pitch() > target + 2.0f && timeout++ < 1000000);
        arm_stop();
    }
    else if (p < target - 2.0f)
    {
        arm_forward();
        while (read_pitch() < target - 2.0f && timeout++ < 1000000);
        arm_stop();
    }
}

// reads the color sensor and shows the raw rgb values on the lcd
// then shows what color it detected for 1 second before returning
// returns 1 for blue 2 for yellow 0 for nothing
uint8_t scan_and_display(void)
{
    uint16_t r, g, b, c;
    tcs_read_rgbc(&r, &g, &b, &c);

    char buf1[16];
    char buf2[16];
    sprintf(buf1, "R:%u G:%u", r, g);
    sprintf(buf2, "B:%u C:%u", b, c);
    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring(buf1);
    LCDgotoXY(0, 1);
    LCDstring(buf2);
    _delay_ms(2000);

    uint8_t color = 0;
    if (c > 100)
    {
        if ((b > r) && (b > g))
        {
            color = 1;
        }
        else
        {
            uint32_t total = r + g + b;
            if (total > 0)
            {
                float rf = (float)r / total;
                float gf = (float)g / total;
                float bf = (float)b / total;
                if (rf > 0.35f && gf > 0.35f && bf < 0.20f)
                {
                    color = 2;
                }
            }
        }
    }

    LCDclr();
    LCDgotoXY(0, 0);
    if (color == 1)       LCDstring("BLUE!");
    else if (color == 2)  LCDstring("YELLOW!");
    else                  LCDstring("No color");
    _delay_ms(1000);

    return color;
}

// drives to a block scans it grabs it if its the right color
// then turns 180 and drives back to the center and drops it
void visit_and_return(float block_heading)
{
    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring("Driving...");
    drive_forward(DRIVE_TIME);

    uint8_t color = scan_and_display();

    uint8_t grabbed = 0;

    if (color > 0)
    {
        // back up a little to get to the right grabbing distance
        LCDclr();
        LCDgotoXY(0, 0);
        LCDstring("Backing up...");
        int start = timer_count;
        motorA_reverse();
        motorB_reverse();
        while ((timer_count - start) < 600);
        robot_stop();

        // arms are already at pitch 18 which is the grab position
        LCDclr();
        LCDgotoXY(0, 0);
        LCDstring("Grabbing...");
        move_arms_to_pitch(PITCH_DRIVE);

        // lift arms up to carry the block back
        LCDclr();
        LCDgotoXY(0, 0);
        LCDstring("Lifting...");
        move_arms_to_pitch(PITCH_CARRY);
        grabbed = 1;
    }

    // turn around and head back to center
    float return_heading = fmodf(block_heading + 180.0f, 360.0f);
    turn_to_heading(return_heading);

    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring("Returning...");
    drive_forward(DRIVE_TIME);

    // drop the block at center if we picked one up
    if (grabbed)
    {
        LCDclr();
        LCDgotoXY(0, 0);
        LCDstring("Dropping...");
        move_arms_to_pitch(PITCH_DRIVE);
    }
}

int main(void)
{
    // xshut needs to be high before anything touches i2c
    PORTA.DIRSET   = PIN4_bm;
    PORTA.OUTSET   = PIN4_bm;
    PORTA.PIN4CTRL |= PORT_PULLUPEN_bm;
    _delay_ms(100);

    init_clock();

    LCDinitialize();
    _delay_ms(500);

    // put bno055 into ndof fusion mode so we get heading and pitch
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x3D);
    TWI_Transmit_Data(0x0C);
    TWI_Stop();
    _delay_ms(800);

    // check bno055 is actually responding before we do anything
    TWI_Address(BNO_ADDR, TW_WRITE);
    TWI_Transmit_Data(0x00);
    TWI_Stop();
    uint8_t bno_id = TWI_Host_Read(BNO_ADDR);
    char ibuf[16];
    sprintf(ibuf, "BNO:0x%02X", bno_id);
    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring(ibuf);
    _delay_ms(2000);

    uart_init(3, 9600, NULL);
    InitADC();
    tcs_init();
    sei();
    InitTCA1();
    InitMotorPWM();
    InitMotorPins();
    PORTB.PIN2CTRL |= PORT_PULLUPEN_bm;

    // record where we are facing at startup
    // the 4 blocks are at 0 90 180 and 270 degrees from this heading
    float start_heading = read_heading();
    float block_headings[4];
    block_headings[0] = fmodf(start_heading,          360.0f);
    block_headings[1] = fmodf(start_heading +  90.0f, 360.0f);
    block_headings[2] = fmodf(start_heading + 180.0f, 360.0f);
    block_headings[3] = fmodf(start_heading + 270.0f, 360.0f);

    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring("My Name is Tamy");
    LCDgotoXY(0, 1);
    LCDstring("Press my button!");

    while (!BTN1);
    _delay_ms(200);
    while (BTN1);
    _delay_ms(200);

    // lower arms to driving position before starting
    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring("Lowering arms...");
    move_arms_to_pitch(PITCH_DRIVE);

    // visit all 4 blocks one by one clockwise
    // block 0 is straight ahead so no turn needed
    for (uint8_t i = 0; i < 4; i++)
    {
        char buf[16];
        sprintf(buf, "Block %d of 4", i + 1);
        LCDclr();
        LCDgotoXY(0, 0);
        LCDstring(buf);
        _delay_ms(1000);

        if (i > 0)
        {
            turn_to_heading(block_headings[i]);
        }

        visit_and_return(block_headings[i]);
    }

    LCDclr();
    LCDgotoXY(0, 0);
    LCDstring("All done!");
    LCDgotoXY(0, 1);
    LCDstring("Tamy finished!");

    while(1){}
    return 0;
}
