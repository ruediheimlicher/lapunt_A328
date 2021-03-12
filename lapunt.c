//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "defines.h"

#include "lcd.c"
//#include "adc.c"


uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

#define OSZIPORT   PORTD      // Ausgang fuer Servo
#define OSZIDDR   DDRD

#define OSZIA 6            // 
#define OSZIB 7            // 
#define OSZIALO OSZIPORT &= ~(1<<OSZIA)
#define OSZIAHI OSZIPORT |= (1<<OSZIA)
#define OSZIATOG OSZIPORT ^= (1<<OSZIA)

#define OSZIBLO OSZIPORT &= ~(1<<OSZIB)
#define OSZIBHI OSZIPORT |= (1<<OSZIB)
#define OSZIBTOG OSZIPORT ^= (1<<OSZIB)


#define LOOPLED_PORT   PORTD
#define LOOPLED_DDR   DDRD
#define LOOPLED_PIN   4

#define INT0_PORT     PORTD  
#define INT0_DDR     DDRD     
#define INT0_PIN     PIND // INT0 auf A8
#define INT0_STATUS  2  // PD2

#define RELAIS_PORT  PORTB
#define RELAIS_DDR   DDRB
#define RELAIS_PIN   PINB

#define RELAIS_ON 0
#define RELAIS_OFF 1

#define RELAIS_ENABLE   2

#define IMPULS_WAIT  16
#define IMPULS_MAX   24

#define ANZAHL_IMPULSE 3

volatile uint8_t   INT0status=0x00;   

volatile uint8_t INT0counter = 0; // Anzahl impulse auf Spule

volatile uint8_t impulscounter = 0; // Anzahl impulse an relais: Sicher ist sicher

void slaveinit(void)
{
   LOOPLED_DDR |= (1<<LOOPLED_PIN);
   
   OSZIDDR |= (1<<OSZIA);
    OSZIDDR |= (1<<OSZIB);

   //LCD
   DDRB |= (1<<LCD_RSDS_PIN);   //Pin 5 von PORT B als Ausgang fuer LCD
    DDRB |= (1<<LCD_ENABLE_PIN);   //Pin 6 von PORT B als Ausgang fuer LCD
   DDRB |= (1<<LCD_CLOCK_PIN);   //Pin 7 von PORT B als Ausgang fuer LCD


   RELAIS_DDR  |= (1<<RELAIS_ENABLE); // output
   RELAIS_PORT |= (1<<RELAIS_ENABLE);// HI

   RELAIS_DDR  |= (1<<RELAIS_ON); // output
   RELAIS_PORT &= ~(1<<RELAIS_ON);// LO
   
   
   RELAIS_DDR  |= (1<<RELAIS_OFF);// Output
   RELAIS_PORT &= ~(1<<RELAIS_OFF);// LO
   
   INT0_DDR &= ~(1<<INT0_STATUS);
   INT0_PORT |= (1<<INT0_STATUS);
   
   ADCSRA = 0;
//   MCUCR = 0;
   SMCR |= (1<<SE);
  SMCR |= (1<<SM1);
   SMCR |= (1<<SM0);
   
}

#pragma mark Timer2
void timer2 (uint8_t wert) 
{ 
   TCCR2B |= (1<<CS20);            //8-Bit Timer, Timer clock = system clock/256
   TCCR2B |= (1 << CS21); 
   TCCR2B |= (1 << CS22); 
   TCCR2A |= (1<<WGM21);      //   ClearTimerOnCompareMatch CTC

   TIFR2 |= (1<<TOV2);             //Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK2 |= (1<<OCIE2A);         //CTC Interrupt aktivieren
   TIMSK2 |= (1 << TOIE2);
   TCNT2 = 0x00;               //Zaehler zuruecksetzen
   
   //;
   OCR2A = 10;               //Setzen des Compare Registers auf HIimpulsdauer
} 

ISR(TIMER2_OFV_vect)
{
   OSZIBTOG;
}

#pragma mark ISR Timer2
ISR(TIMER2_COMPA_vect) // Schaltet Impuls an MOTOROUT LO wenn speed
{
 //  OSZIATOG;
   //return;
    if (INT0status & (1<<1))  // interrupt am laufen
    {
       INT0counter++;
       if (INT0counter > IMPULS_WAIT)
       {
          if (INT0_PIN & (1<<INT0_STATUS)) // Pin ist HI
           {
              RELAIS_PORT |= (1<<RELAIS_ON); // ON-Spule EIN
           }
           else
           {
              RELAIS_PORT |= (1<<RELAIS_OFF);// ON-Spule EIN
           }
       }
       
       if (INT0counter > IMPULS_MAX)// Ereignis fertig
       {
//          INT0status &= ~(1<<1);
          RELAIS_PORT &= ~(1<<RELAIS_ON); // ON-Spule AUS
          RELAIS_PORT &= ~(1<<RELAIS_OFF); // ON-Spule AUS
          
 //         RELAIS_PORT |= (1<<RELAIS_ENABLE); // enable OFF
          
          INT0counter = 0; // reset auf WAIT, Pause zwischen impulsen
          impulscounter ++;
       }
       
       
       if (impulscounter >= ANZAHL_IMPULSE)
       {
          INT0status &= ~(1<<1);
          impulscounter = 0;
          RELAIS_PORT |= (1<<RELAIS_ENABLE); // enable OFF
          //        MCUCR |= (1<<SM1);
          //        MCUCR |= (1<<SM0);
          //MCUCR = 0;
       }
    }
} // TIMER0_COMPA_vect

#pragma mark INT0
void int0_init(void)
{
   
   EICRA |= (1<<ISC00 ); // raise int0 on any Change
   EIMSK |= (1<<INT0); // enable external int0
   INT0status = 0;
   INT0counter = 0; // begin erster impuls vor WAIT
   
}


ISR(INT0_vect) 
{
   INT0status |=(1<<1);
   RELAIS_PORT &= ~(1<<RELAIS_ENABLE);
   OSZIBTOG;

}


void main (void) 
{
   /* INITIALIZE */
   
//   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
//   lcd_puts("Guten Tag\0");
//   lcd_cls();
//   lcd_puts("READY\0");
   timer2(4); // 128 ca. 130ms
   
   int0_init();
   
   slaveinit();
   
//   lcd_gotoxy(16,0);
//   _delay_ms(100);
//   int i=0;
   sei();
   
//   MCUCR |= (1<<SE);
//   MCUCR |= (1<<SM1);
//   MCUCR |= (1<<SE);

//  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
#pragma mark while
   while (1) 
   {
      //OSZIBTOG;
      
      loopCount0 ++;
      if (loopCount0 >=0x00FF)
      {
         LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         loopCount1++;
         
         if ((loopCount1 >0x0008) )
         {
            loopCount1=0;
            uint8_t index=0;

         }
         
         loopCount0 =0;
      }
      
   }
   
   
   return 0;
}
