#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define delay 6  /* Delay between each 7-segment toggle */

unsigned char num = 0;
unsigned flag = 0;

typedef struct time{
	unsigned char sec1,sec2,min1,min2,hour1,hour2;
}time;

struct time t; /* Global Structure */

void Timer1_Init(void)
{
	OCR1A = 976; /* 1000/1024/1000 = 976.5 */
	TIMSK |= (1<<OCIE1A); /* Timer1 interrupt flag on */
	TCCR1A = (1<<FOC1A); /* Non-PWM Mode */
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10); /* CTC mode - 1024 pre-scaler */
}
void countt1(void){
			if(t.sec1<9){
				t.sec1++;}
			else{
				t.sec1=0;
			 if(t.sec2<5){
				t.sec2++;}
			 else{
				t.sec2=0;
			  if(t.min1<9){
				t.min1++;}
			  else{
				t.min1=0;
			   if(t.min2<5){
				t.min2++;}
			   else{
				t.min2=0;
				if(t.hour1<5){
				t.hour1++;}
			    else{
			    t.hour1=0;
			    if(t.hour2<5){
			    t.hour2++;}}}}}}}

ISR(TIMER1_COMPA_vect)
{
		flag = 1; /* If one second has passed, set flag */
}
ISR(INT0_vect){
	t.sec1 = 0;
	t.sec2 = 0;
	t.min1 = 0;
	t.min2 = 0;
	t.hour1 = 0;
	t.hour2 = 0;
}
ISR(INT1_vect){
	TIMSK &= ~(1<<OCIE1A); /* Turn off interrupt flag */
}
ISR(INT2_vect){
	TIMSK |= (1<<OCIE1A); /* Turn on interrupt flag */
}

void Interrupt_Init(void){
	DDRB &= ~(0x04); /* PB2 Input -> INT2  */
	DDRD &= ~(0x04); /* PD2 Input -> INT0  */
	DDRD &= ~(0x08); /* PD3 Input -> INT1  */
	PORTB |= (1<<PB2); /* Internal pull-up */
	PORTD |= (1<<PD2); /* Internal pull-up */
	MCUCR = (1<<ISC01)|(1<<ISC11)|(1<<ISC10); /* Falling edge interrupt on INT0, falling edge interrupt on INT1 */
	MCUCSR &= ~(1<<ISC2); /* Falling edge interrupt on INT2 */
	GICR = 0xE0; /* Enable INT0, INT1, and INT2 */
	SREG  |= (1<<7); /* Global interrupt flag on */

}

int main(void)
{
	DDRC  |= 0x0F;
	PORTC &= 0xF0;
	DDRA |= 0x3F;
	PORTA &= 0xF0;
	Timer1_Init();
	Interrupt_Init();


    while(1)
    {
    	if(flag){
    		countt1();
    		flag = 0;
    	}
    	PORTA = 0x01;
    	PORTC = t.hour2;
    	_delay_ms(delay);
    	PORTA = 0x02;
    	PORTC = t.hour1;
    	_delay_ms(delay);
    	PORTA = 0x04;
    	PORTC = t.min2;
    	_delay_ms(delay);
    	PORTA = 0x08;
    	PORTC = t.min1;
    	_delay_ms(delay);
        PORTA = 0x10;
        PORTC = t.sec2;
        _delay_ms(delay);
        PORTA = 0x20;
        PORTC = t.sec1;
        _delay_ms(delay);
        }
}
