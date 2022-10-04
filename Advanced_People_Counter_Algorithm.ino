/**
   @file Advanced_People_Counter_Algorithm.ino
   @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
   @brief This is a program for counting people entering or exiting
   a place/door.
   This algorithm uses pin change interrupts to enable 
   accurate people count.
   By accounting for pin change pulses (both leading and trailling edges) 
   we can get the total people count in a place while avoiding errors 
   such as:
   - Synchronization problem
   - Invalid counts
   - unstable entry and exit speeds
   @version 0.3
   @date 2022-09-4

   @copyright Copyright (c) 2022

*/
//#define F_CPU 12000000UL //Arduino mega clock frequency

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

int8_t int_flag_1 = 0;
int8_t int_flag_2 = 0;
//int8_t valid_int_flag_1 = 0;
//int8_t valid_int_flag_2 = 0;
int8_t int_last_state_1 = 0;
int8_t int_last_state_2 = 0;
unsigned long timer1_buf = 0;
volatile int count = 0;

#define DELAY 50
#define BAUDRATE 9600
#define SIZE 100
char buf[SIZE] = "Hello World!";
#define ubbr  ((F_CPU) /(16*BAUDRATE) - 1)

int main(void) {
  //Serial.begin(9600); // Uncomment for debugging purposes
  //    UBRR0H = (uint8_t) (ubbr >> 8);
  //    UBRR0L = (uint8_t) (ubbr);
  //    //    UBRR0 = 103;
  //    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  //    //    UCSR0C |= _BV(USBS0) | (3 << UCSZ00);
  //    UCSR0C |= _BV(UCSZ00) | _BV(UCSZ01);
  //DDRB |= _BV(PB7); // Builtin LED (Arduino mega)
  // PORTB &= ~(_BV(PB7)); //turning the LED off
  //  PCICR |= 0X01;

  DDRB |= _BV(PB2); // Power LED ==> output
  DDRB |= _BV(PB3); // Working LED ==> output
  DDRB |= _BV(PB6); // Relay ==> output
  DDRB |= _BV(PB5); // Builtin LED ==> output // testing
  
  PORTB |= ~(_BV(PB2)); // Power LED Off
  PORTB &= ~(_BV(PB5)); // Builtin LED Off // testing 
  PCICR |= (1 << PCIE0); // Initializing the Pin Change Interrupt register
  //  PCMSK0 |= B00000110;
  PCMSK0 |= (1 << PCINT1); //PCMSK0 |= (1 << PCINT1) //for the Arduino mega test
  PCMSK0 |= (1 << PCINT4); //PCMSK0 |= (1 << PCINT2) //for the Arduino mega test
  DDRB |= _BV(PB7);
  sei();
  PORTB |= _BV(PB2); // Power LED On
  
  while (1) {
    count < 0 ? count = 0 : count;
  // check if count is not equal to zero
    if (count >= 1) {
      // turn on working LED and relay
      PORTB |= _BV(PB3);// Working LED
      PORTB |= _BV(PB6);// Relay
    }
    else if(count == 0)
    {
      // turning off Working LED and Relay
      PORTB &= ~(_BV(PB3));// Working LED
      PORTB &= ~(_BV(PB6));// Relay
    }
    
    // Serial.println(count); // for debugging purposes

//    ATOMIC_BLOCK(ATOMIC_FORCEON) {
//      if (int_flag) {
//        PORTB ^= _BV(PB7);
//        int_flag = 0;
//        
//      }
//      
//
//
//    }
    //         for (uint8_t i = 0; i < SIZE; i++) {
    //                    while (!(UCSR0A & _BV(UDRE0)));
    //                    UDR0 = buf[i];
    //                    UCSR0A |= _BV(TXC0);
    //                }
    //        PORTB |= _BV(PB7);
    //        _delay_ms(500);
    //        PORTB &= ~(_BV(PB7));
    //        _delay_ms(500);
  }
}

// Using pin change interrupt service routine
ISR(PCINT0_vect) {
  // The algorithm is to use a leading edge PC interrupt to initiate
  // an interrupt flag and a trailing egde to validate it

  //entry sensor
  if (PINB & _BV(PB4)) { // if (PINB & _BV(PB1)) (for Arduino mega test)
    if (int_last_state_1 == 0) {
      int_last_state_1 = 1;
    }
  }
  else if(int_last_state_1 == 1){
    int_last_state_1 = 0;
    if (int_flag_1 == 0 && int_flag_2 == 0) {
      int_flag_1 = 1;
    }
    else if (int_flag_1 == 0 && int_flag_2 == 1) {
      count--;
      int_flag_2 = 0;
    }
  }

  //exit sensor
  if (PINB & _BV(PB1)) { // if (PINB & _BV(PB2)) (for Arduino mega test)
    if (int_last_state_2 == 0) {
      int_last_state_2 = 1;
    }
  }
  else if(int_last_state_2 == 1){
    int_last_state_2 = 0;
    if (int_flag_1 == 0 && int_flag_2 == 0) {
      int_flag_2 = 1;
    }
    else if (int_flag_1 == 1 && int_flag_2 == 0) {
      count++;
      int_flag_1 = 0;
    }
  }

//  _delay_ms(DELAY); // debounce
}







//int8_t int_flag_1 = 0;
//int8_t int_last_state = 0;
//unsigned long timer1_buf = 0;
//int count = 0;
//
//void setup() {
//  Serial.begin(9600);
//
//  //if (PCIFR & B00000001){
//  //  Serial.println("Flag set!");
//  //}
//  DDRB |= _BV(PB7);
//  PORTB &= ~(_BV(PB7));
//  //  PCICR |= 0X01;
//  PCICR |= (1 << PCIE0);
//  //  PCMSK0 |= B00000110;
//  PCMSK0 |= (1 << PCINT1);
//  PCMSK0 |= (1 << PCINT2);
//  sei();
//}
//void loop() {
//  //unsigned long current_time = millis();
//  //
//  //    if(current_time - timer1_buf >= 500){
//  //
//  //      timer1_buf = current_time;
//  //Serial.println(count);
//  //      if(int_flag_1 == 0){
//  //        PORTB |= _BV(PB7);
//  //        int_flag_1 = 1;
//  //      }
//  //      else if(int_flag_1 == 1){
//  //        PORTB &= ~(_BV(PB7));
//  //        int_flag_1 = 0;
//  //      }
//  while (1) {
//    Serial.println(count);
//  }
//
//  //    }
//
//  //    PORTB |= _BV(PB7);
//  //    for (uint32_t t = 0; t < 16000; t++)  //debounce
//  //      for (uint32_t i = 0; i < 54; i++)
//  //      asm("nop");
//  //    PORTB &= ~(_BV(PB7));
//  //    for (uint32_t t = 0; t < 16000; t++)  //debounce
//  //      for (uint32_t i = 0; i < 54; i++)
//  //      asm("nop");
//}
//
//
//ISR(PCINT0_vect) {
//  if (PINB & _BV(PB1)) {
//    for (int t = 0; t < 65535; t++);  //debounce
//    if (int_last_state == 0) {
//      int_last_state = 1;
//      PORTB |= _BV(PB7);
//    }
//    else if (int_last_state == 1) {
//      int_last_state = 0;
//      PORTB &= ~(_BV(PB7));
//      count++;
//    }
//  }
//}
