/*

TIMER PINS:

ARDUINO MEGA:
TIMER1 - Pins 11 & 12

ARDUINO UNO
TIMER0 - Pins 5 & 6
TIMER1 - Pins 9 & 10 (16bit timer)
TIMER2 - Pins 11 & 3

ARDUINO NANO
TIMER1 - Pins 11 & 12 (16bit timer)

*/

#include "MotorStepper.h"
#include <avr/interrupt.h>

/*
* PIN PORTs and MASKs
*
* This enables to write very fast
* directly to PIN without any functions
*/
volatile uint8_t *STEPPER_PORT_DIR;
volatile uint8_t *STEPPER_PORT_CLK;
volatile uint8_t STEPPER_MASK_DIR;
volatile uint8_t STEPPER_MASK_CLK;

// Timer variables
volatile unsigned int STEPPER_FREQUENCY;
volatile uint8_t STEPPER_TIMER_PRESCALER;

// Stepper mode (Waypoints/Continuous)
volatile byte STEPPER_MODE;
volatile bool STEPPER_DIRECTION;

// Stepper Waypoints
volatile MotorWaypoints STEPPER_WAYPOINTS;

/**
 * Get Waypoints
 * 
 * It's volatile, since it has to be accessed
 * in the OCR function.
 * 
 * OCR doesnt allow access of non-volatile variables
 * 
 * @return 
 */
static volatile MotorWaypoints* MotorStepper::Waypoints() {
    return &STEPPER_WAYPOINTS;
}

static volatile unsigned int MotorStepper::Frequency() {
    return STEPPER_FREQUENCY;
}

static volatile bool MotorStepper::Direction() {
    return STEPPER_DIRECTION;
}


MotorStepper::MotorStepper(int direction_pin, int clock_pin) {
    PIN_DIR = direction_pin;
    PIN_CLK = clock_pin;

    // Volatile Variables
    STEPPER_FREQUENCY = 0;
    STEPPER_TIMER_PRESCALER = 0;
    
    MotorStepper::SetDirection(FORWARD);    
}

/**
 * Initializes Motor
 * 
 * Note, that you cannot use more than 1 motor, since
 * it uses the same volatile variables! 
 * 
 * @param directionInvert
 */
void MotorStepper::Init() {
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_DIR, HIGH);
    digitalWrite(PIN_CLK, HIGH);

    STEPPER_PORT_DIR = portOutputRegister(digitalPinToPort(PIN_DIR));
    STEPPER_PORT_CLK = portOutputRegister(digitalPinToPort(PIN_CLK));

    STEPPER_MASK_DIR = digitalPinToBitMask(PIN_DIR);
    STEPPER_MASK_CLK = digitalPinToBitMask(PIN_CLK);

    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_DIR, LOW);
    
    // Init Timer
    _tInit();
}

/**
 * Initializes Timer 
 * 
 * Timer is separate from the main core
 * and executes the OCR interrupt each time.
 * 
 * We use 16-bit Timer1, since it has more precise control
 */
void MotorStepper::_tInit() {
    noInterrupts(); // disable all interrupts
    TCCR1A = 0; // Clear the mode register completely
    TCCR1B = 0; // Clear the mode register completely
    TCNT1 = 0; // Clear timer counter

    //bitWrite:
    //register|byte index|value

    // Set to CTC (Clear on compare mode)
    bitWrite(TCCR1B, WGM12, 1);

    // Set timer prescaler
    MotorStepper::tSetPrescaler(0);

    // Activate pin toggle on interrupt
    //bitWrite(TCCR1A, COM1A0, 1);
    //bitWrite(TCCR1A, COM1B0, 1);

    // Set timer interrupt: Compare Output A
    bitWrite(TIMSK1, OCIE1A, 1);

    // Set timer compare value
    // (On this value timer should trigger interrupt clear)   
    OCR1A = 0;    

    interrupts(); // enable all interrupts    
}

static byte MotorStepper::tCalcPrescaler(int freq) {
    if (freq >= 1340) {
        return 1;
    }

    if (freq >= 630) {
        return 8;
    }
    
    if (freq > 0) {
        return 64;
    }

    return 0;    
}

/**
 * Calculates and sets the pre-scaler to the timer
 * 
     CSx2  CSx1  CSx0  Divider
     0     0     0     No Clock source (Stopped)
     0     0     1     1/1
     0     1     0     1/8
     0     1     1     1/64
     1     0     0     1/256
     1     0     1     1/1024
 */
static void MotorStepper::tSetPrescaler(byte p) {
    switch (p) {
        case 1:
            bitWrite(TCCR1B, CS12, 0); // Set prescaler
            bitWrite(TCCR1B, CS11, 0); // Set prescaler
            bitWrite(TCCR1B, CS10, 1); // Set prescaler
            break;
        case 8:
            bitWrite(TCCR1B, CS12, 0); // Set prescaler
            bitWrite(TCCR1B, CS11, 1); // Set prescaler
            bitWrite(TCCR1B, CS10, 0); // Set prescaler   
            break;
        case 64:
            bitWrite(TCCR1B, CS12, 0); // Set prescaler
            bitWrite(TCCR1B, CS11, 1); // Set prescaler
            bitWrite(TCCR1B, CS10, 1); // Set prescaler     
            break;
            
        default:
            bitWrite(TCCR1B, CS12, 0); // Set prescaler
            bitWrite(TCCR1B, CS11, 0); // Set prescaler
            bitWrite(TCCR1B, CS10, 0); // Set prescaler              
    }
    
    STEPPER_TIMER_PRESCALER = p;
}

/**
 * Stops the timer
 */
static void MotorStepper::tStop() {
    MotorStepper::tSetPrescaler(0);
}

/**
 * Stepper Motor: Set Direction
 */
static void MotorStepper::SetDirection(bool direction) {
    if (STEPPER_DIRECTION == direction) {
        return;
    }
    
    STEPPER_DIRECTION = direction;
    
    #ifdef STEPPER_DIRECTION_INVERT
    if (STEPPER_DIRECTION) {
       *STEPPER_PORT_DIR &= ~STEPPER_MASK_DIR;
    } else {
        *STEPPER_PORT_DIR |= STEPPER_MASK_DIR;  // Set pin high            
    }
    #else
    if (STEPPER_DIRECTION) {
        *STEPPER_PORT_DIR &= ~STEPPER_MASK_DIR;
    } else {
        *STEPPER_PORT_DIR |= STEPPER_MASK_DIR;  // Set pin high            
    }
    #endif
}

static bool MotorStepper::IsRunning() {
    return STEPPER_TIMER_PRESCALER ? true : false;
}

static void MotorStepper::SetFrequency(int freq) {   
    if (freq != STEPPER_FREQUENCY) { //4us
        //Serial.println(freq);
        
        STEPPER_FREQUENCY = freq; //4us

        if (STEPPER_FREQUENCY == 0) {
            MotorStepper::tStop();
            return;
        } else {
            STEPPER_TIMER_PRESCALER = MotorStepper::tCalcPrescaler(freq); //4us

            // We need to assign value to longint
            // because OCR can only store 16bit integer
            // therefore causing trouble if STEPPER_FREQUENCY 
            // is too low
            unsigned long ocr;
            
            #ifdef STEPPER_FAST_OVER_PRECISE
            if (STEPPER_FREQUENCY < 1340) {
                ocr =  F_CPU / (((long) STEPPER_FREQUENCY) * ((long)STEPPER_TIMER_PRESCALER) * 2) - 1; //44us
            } else {
                // We use linear fit instead. It's not precise hertz-wise
                // but smooth. Saves time
                ocr = 6368 - 0.298 * STEPPER_FREQUENCY; //24us
            }
            #else 
            ocr =  F_CPU / (((long) STEPPER_FREQUENCY) * ((long) prescaler) * 2) - 1; //44us
            #endif

            OCR1A = ocr > 65535 ? 65535 : ocr;

            if (TCNT1 >= OCR1A) {
                TCNT1 = TCNT1 - OCR1A;
            }
                       
            if (MotorStepper::IsRunning()) {
                // Set prescaler only if motor is running
                // this allows to pre-set frequency before hitting run
                MotorStepper::tSetPrescaler(STEPPER_TIMER_PRESCALER);
            }
        }   
    }
}

static void MotorStepper::Run() {
    if (STEPPER_MODE == MODE_CONTINOUS) {
        if (STEPPER_FREQUENCY == 0) {
            MotorStepper::Stop();
            return;
        }       
    }

    if (!MotorStepper::IsRunning()) {
        if (STEPPER_MODE == MODE_WAYPOINT) {
            MotorStepper::SetFrequency(50);
            STEPPER_WAYPOINTS.Start();
        }            
        
        TCNT1 = 0; // Resets timer count
        // We want to set frequency before starting the motor, so:
        // Timer was possibly shut down, so we need to start it
        // SetFrequency doesnt start a shut down timer
        MotorStepper::tSetPrescaler(STEPPER_TIMER_PRESCALER);        
    }
}

static void MotorStepper::Stop() {
    if (MotorStepper::IsRunning()) {
        *STEPPER_PORT_CLK &= ~STEPPER_MASK_CLK;
        MotorStepper::tStop();
    }
    
    if (STEPPER_MODE == MODE_WAYPOINT) {
        STEPPER_WAYPOINTS.Stop();
    }
}

static void MotorStepper::SetMode(uint8_t m) {
    STEPPER_MODE = m;
}

static void MotorStepper::Step() {
    *STEPPER_PORT_CLK ^= STEPPER_MASK_CLK; // Do a step    
}

/**
 * This function runs on each timer compare success
 * AKA Interrupt function
 * 
 * interrupt service routine that wraps a user defined function supplied by attachInterrupt
 */
ISR(TIMER1_COMPA_vect) 
{
    if (STEPPER_MODE == MODE_CONTINOUS) {
        MotorStepper::Step();
        return;
    }

    if (STEPPER_MODE == MODE_WAYPOINT) {
        STEPPER_WAYPOINTS.Tick();
    }
}