#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#define RS PB0 //En pin is connected to PORTB Pin 0
#define RW PB1 //En pin is connected to PORTB Pin 1
#define EN PB2 //En pin is connected to PORTB Pin 2
int LCD_init(void);
void LCD_Send_Command(unsigned char);
void LCD_Send_Data(unsigned char);
int LCD_Send_Array(char * ptr);
int main(void)
{

DDRC |= (1<<PC0);  //Nakes first pin of PORTC as Output
  // OR DDRC = 0x01;
  DDRD &= ~(1<<PD0);//Makes firs pin of PORTD as Input
  // OR DDRD = 0x00; //Makes all pins of PORTD input
  while(1) //infinite loop
  {
    if(PIND & (1<<PD0) == 1) //If switch is pressed
    {
     LCD_init();
    LCD_Send_Array("Hajra Fatima"); /* Replace with your application code */
    }  
    else{
LCD_init();
LCD_Send_Array("FA19-BCE-072"); /* Replace with your application code */
  }   
  }
  
  while (1) {
}
return(0);
}
int LCD_init() {
DDRD = 0b11111111;
DDRB = 0b00000111;
UCSR0B&=~(1<<TXEN0);
UCSR0B&=~(1<<RXEN0);
_delay_ms(100);
PORTB |= (1<<EN);
LCD_Send_Command(0x38);
_delay_ms(2);
LCD_Send_Command(0x0E);
_delay_ms(2);
LCD_Send_Command(0x01);
_delay_ms(2);
LCD_Send_Command(0x06);
return(0);
}
void LCD_Send_Command(unsigned char comm)
{
PORTB &= ~(1<<RS);
PORTB &= ~(1<<RW);
PORTD = comm;
PORTB &= ~(1<<EN);
_delay_ms(1);
PORTB |= (1<<EN);
}
void LCD_Send_Data(unsigned char data)
{
PORTB |= (1<<RS);
PORTB &= ~(1<<RW);
PORTD = data;
PORTB &= ~(1<<EN);
_delay_ms(1);
PORTB |= (1<<EN);
}
int LCD_Send_Array(char * ptr)
{
while(*ptr != '\0')
{
LCD_Send_Data(*ptr);
ptr++;
} }
