/*
                      Step Sequencier
                      Vernon Billingsley c2021

                      8 Step Sequencer for the
                      arduino Uno, Nano

                Pin     Function
                2       Clock Input
                3       Run/Manual
                4       Manual Step Button
                6       Gate LED
                7       Gate Ouput

                8 |           Control A
                9 | CD4051    Control B
                10|           Control C

                AREF          .1 uF cap to ground
                A0            Step Direction
                                  Step Up, Step Down,
                                  Step Up then Down,
                                  Change direction based on
                                    a prime random number

                A1            Count to N


    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission
    notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#include <elapsedMillis.h>

/************************* Defines ********************************/
#define DEBUG 1

#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define interrupt_pin 2

/************************** Variables *****************************/
int16_t step_count = 0;

uint8_t adc_avg;
uint8_t adc_step;

boolean adc_zero = true;

boolean step_up = true;
boolean next_step = false;
boolean rnd_up = true;

boolean stp_up = true;
boolean stp_dn = false;
boolean stp_up_dn = false;
boolean stp_rand = false;

uint8_t max_count = 7;
uint8_t min_count = 0;

const uint8_t prime_array[] = {
  2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31,
  37, 41, 43, 47, 53, 59, 61, 67, 71, 73,
  79, 83, 89, 97,
};

/**************************  Functions ****************************/


/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }


  /************************* Setup Pins ***************************/
  /*Interrupt Pin */
  DDRD &= ~_BV (2); // pinMode (2, INPUT);
  /*Run, manual step */
  DDRD &= ~_BV (3); // pinMode (3, INPUT);
  /*Manual step button */
  DDRD &= ~_BV (4);
  /*Gate LED */
  DDRD |= _BV (6); // pinMode (6, OUTPUT);
  /*Gate Pin */
  DDRD |= _BV (7); // pinMode (7, OUTPUT);

  DDRB |= _BV (0); // pinMode (8, OUTPUT);
  DDRB |= _BV (1); // pinMode (9, OUTPUT);
  DDRB |= _BV (2); // pinMode (10, OUTPUT);

  /******************** Clock Interrupt ************************/
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), CLK_ISR, CHANGE);

  /*************************  Setup ADC ***************************/
  /*Left Adjust for 255 precision */
  sbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  /*Set to ADC0 to start */
  cbi(ADMUX, MUX3);
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  cbi(ADMUX, MUX0);

  /*Set prescaler to 128 */
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  sbi(ADCSRA, ADEN);

  /*Start the first conversion */
  sbi(ADCSRA, ADSC);

}/**************************  End Setup **************************/


void CLK_ISR() {
  /*Read the state of the interrupt pin */
  uint8_t pin_state = (PIND & _BV (2));
  if (pin_state) {
    PORTB = step_count;
    PORTD |= _BV (6); // digitalWrite (6, HIGH);
    PORTD |= _BV (7); // digitalWrite (7, HIGH);
    next_step = true;
  } else {
    PORTD &= ~_BV (6); // digitalWrite (6, LOW);
    PORTD &= ~_BV (7); // digitalWrite (7, LOW);
  }
}

elapsedMillis check_adc;
/*Debounce the button */
elapsedMillis button_time;

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {
  /*Check for run or manual mode */
  uint8_t mode = (PIND & _BV (3));

  /************** Run Mode ***************/
  /*Pin 3 High */

  if (mode) {
    /*Allow interrupts */
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), CLK_ISR, CHANGE);
    if (check_adc > 100) {
      /*Check to see if ADC has finished */
      if (!(bitRead(ADCSRA, ADSC))) {
        /*Read the High byte */
        uint8_t temp_adc = ADCH;
        /*Keep a running average */
        if (adc_zero) {
          adc_avg = (adc_avg + temp_adc) / 2;
        } else {
          adc_step = (adc_step + temp_adc) / 2;
        }
        /*Change the MUX */
        adc_zero = !adc_zero;
        cli();                //stop interrupts
        if (adc_zero) {
          cbi(ADMUX, MUX3);
          cbi(ADMUX, MUX2);
          cbi(ADMUX, MUX1);
          cbi(ADMUX, MUX0);
        } else {
          cbi(ADMUX, MUX3);
          cbi(ADMUX, MUX2);
          cbi(ADMUX, MUX1);
          sbi(ADMUX, MUX0);
        }
        /*Start the next conversion */
        sbi(ADCSRA, ADSC);
        sei();                //allow interrupts

        if (adc_zero) {
          /*Determine the count direction */
          switch (adc_avg) {
            case 0 ... 63:
              stp_up = true;
              stp_dn = false;
              stp_up_dn = false;
              stp_rand = false;
              break;
            case 64 ... 127:
              stp_up = false;
              stp_dn = true;
              stp_up_dn = false;
              stp_rand = false;
              break;
            case 128 ... 190:
              stp_up = false;
              stp_dn = false;
              stp_up_dn = true;
              stp_rand = false;
              break;
            case 191 ... 255:
              stp_up = false;
              stp_dn = false;
              stp_up_dn = false;
              stp_rand = true;
              break;
          }/*End direction */

        } else {
          switch (adc_step) {
            case 0 ... 31:
              max_count = 0;
              break;
            case 32 ... 63:
              max_count = 1;
              break;
            case 64 ... 95:
              max_count = 2;
              break;
            case 96 ... 127:
              max_count = 3;
              break;
            case 128 ... 159:
              max_count = 4;
              break;
            case 160 ... 191:
              max_count = 5;
              break;
            case 192 ... 223:
              max_count = 6;
              break;
            case 224 ... 255:
              max_count = 7;
              break;
          }/*End adc step */
        }
      }
      /*Reset the adc timer */
      check_adc = 0;
    }

    if (next_step) {
      if (stp_up) {
        step_count++;
        if (step_count > max_count) {
          step_count = min_count;
        }
      }

      if (stp_dn) {
        step_count--;
        if (step_count < min_count) {
          step_count = max_count;
        }
      }

      if (stp_up_dn) {
        if (!step_up) {
          step_count++;
          if (step_count > max_count) {
            step_count = max_count;
            step_up = true;
          }
        }

        if (step_up) {
          step_count--;
          if (step_count < min_count) {
            if (max_count > 0) {
              step_count = min_count + 1;
            } else {
              step_count = max_count;
            }
            step_up = false;
          }

        }
      }/*Step up down */

      if (stp_rand) {
        if (!rnd_up) {
          step_count++;
          if (step_count > max_count) {
            step_count = max_count;
            rnd_up = true;
          }
        }

        if (rnd_up) {
          step_count--;
          if (step_count < min_count) {
            if (max_count > 0) {
              step_count = min_count + 1;
            } else {
              step_count = max_count;
            }
            rnd_up = false;
          }

        }
        /*Every loop draw a random number between 1 and 100 */
        uint8_t r_num = random(1, 100);
        /*Check to see if it's prime */
        for (uint8_t x = 0; x < sizeof(prime_array); x ++) {
          if (r_num == prime_array[x]) {
            rnd_up = !rnd_up;
          }
        }
      }/*Step Rando up down */

      next_step = false;
    }
  }/****************** End Run Mode *****************/

  /******************* Manual Step Mode *************/

  if (!mode) {
    /*Stop the interrupt */
    detachInterrupt(digitalPinToInterrupt(interrupt_pin));
    /*Turn on the Gate */
    PORTD |= _BV (6); // digitalWrite (6, HIGH);
    PORTD |= _BV (7); // digitalWrite (7, HIGH);
    /*Get the state on the button */
    uint8_t button_high = (PIND & _BV (4));
    /*If the button is pressed */
    if (button_high && button_time > 200) {
      step_count++;
      if (step_count > 7) {
        step_count = 0;
      }
      PORTB = step_count;
      /*Reset the button timer */
      button_time = 0;
    }

  }/*End manual mode */

}/*************************** End Loop *****************************/
