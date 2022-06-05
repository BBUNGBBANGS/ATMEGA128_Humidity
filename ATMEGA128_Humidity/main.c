

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

uint8_t DHT11_I_RH,DHT11_D_RH,DHT11_I_TEMP,DHT11_D_TEMP,DHT11_CheckSum;
char tx_data[32];

void Port_Init(void)
{
    DDRA = 0xFF; //PA0~PA7 Output Direction
    DDRB = 0x20; //PB5 Output Direction
    DDRD = 0x07; //PD0~2 Output Direction
}

void Timer1_Init(void)
{
    TCCR1A = 0x82;  // FAST PWM Mode
    TCCR1B = 0x1A; // FAST PWM MODE(10bit)
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1 = 39999; // 2[MHz] / (1 + 39999) = 50[Hz] -> 20[ms]
    OCR1A = 3999; // 20[ms] * 0.1 : 2[ms] : Servo Motor 90[Degree]
}

void ISR_Init(void)
{
    SREG = 0x80; // Global Interrupt Enable
    return;
}

void LCD_Init(void)
{
	LCD_Transmit_Command(0x38);
	_delay_ms(10);
	// 비지 플래그를 체크하지 않는 Function Set
	LCD_Transmit_Command(0x38);
	_delay_us(200);
	// 비지 플래그를 체크하지 않는 Function Set
	LCD_Transmit_Command(0x38);
	_delay_us(200);
	
	// 비지 플래그를 체크하는 Function Set
	LCD_Transmit_Command(0x38);
	// 비지 플래그를 체크하는 Display On/Off Control
	LCD_Transmit_Command(0x0c);
	// 비지 플래그를 체크하는 Clear Display
	LCD_Transmit_Command(0x01);
}

void DHT11_Transmit_Request(void)
{
	DDRD |= (1<<DHT11_PIN); // PD6 Output Setting
	PORTD &= ~(1<<DHT11_PIN); // PD6 Low
	_delay_ms(20);
	PORTD |= (1<<DHT11_PIN); // PD6 High
}

void DHT11_Response(void)
{
	DDRD &= ~(1<<DHT11_PIN); // PD6 Input Setting
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data(void)
{
	uint8_t data = 0;
	for (int i = 0; i < 8; i++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);  	/* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN)) 				/* if high pulse is greater than 30ms */
		{
			data = (data<<1)|(0x01);					/* then its logic HIGH */
		}
		else									/* otherwise its logic LOW */
		{
			data = (data<<1);
		}

		while(PIND & (1<<DHT11_PIN));
	}
	return data;
}

int main(void)
{
    uint8_t Calc_CheckSum = 0;
	uint8_t data = 0;
	uint8_t length = 0;
	uint8_t n1,n10,n100;
	char Humidity;
	char temperature;

    Port_Init();
    LCD_Init();
    ISR_Init();

    while (1) 
    {
        DHT11_Transmit_Request(); //DHT11 data Transmit Reqeust
        DHT11_Response(); //DHT11 Response holding

        DHT11_I_RH = Receive_data();
        DHT11_D_RH = Receive_data();
        DHT11_I_TEMP = Receive_data();
        DHT11_D_TEMP = Receive_data();
        DHT11_CheckSum = Receive_data();

        Calc_CheckSum = DHT11_I_RH + DHT11_D_RH + DHT11_I_TEMP + DHT11_D_TEMP;
        if(Calc_CheckSum == DHT11_CheckSum)
        {
			for(int i = 0; i<16;i++)
			{
				Clear_Buffer(0,i);
				LCD_Cursor(0,i);
				sprintf(&tx_data[0],"Humidity : ");
				length = strlen(tx_data);
				n100 = (DHT11_I_RH / 100) % 10;
				n10 = (DHT11_I_RH / 10) % 10;
				n1 = (DHT11_I_RH % 10);
				tx_data[length] = n100;
				length = length + 1;
				tx_data[length] = n10;
				length = length + 1;
				tx_data[length] = n1;
				length = length + 1;
				n10 = (DHT11_D_RH / 10) % 10;
				n1 = (DHT11_D_RH % 10);
				tx_data[length] = n10;
				length = length + 1;
				tx_data[length] = n1;
				LCD_Transmit_Data(tx_data[i]);
			}
			length = 0;
			for(int i = 0;i<16;i++)
			{
				Clear_Buffer(1,i);
				LCD_Cursor(1,i);
				sprintf(tx_data[length],"Temperature : ");
				length = strlen(tx_data);
				n100 = (DHT11_I_TEMP / 100) % 10;
				n10 = (DHT11_I_TEMP / 10) % 10;
				n1 = (DHT11_I_TEMP % 10);
				tx_data[length] = n100;
				length = length + 1;
				tx_data[length] = n10;
				length = length + 1;
				tx_data[length] = n1;
				length = length + 1;
				n10 = (DHT11_D_TEMP / 10) % 10;
				n1 = (DHT11_D_TEMP % 10);
				tx_data[length] = n10;
				length = length + 1;
				tx_data[length] = n1;
				LCD_Transmit_Data(tx_data[i+16]);
			}

            if( DHT11_I_RH >= 40)
            {
                OCR1A = 2999; // 20[ms] * 0.075 : 1.5[ms]  ==> Servo Motor 0[Degree]
            }
            else
            {
                OCR1A = 3999; // 20[ms] * 0.1 : 2[ms] : Servo Motor 90[Degree]
            }
        }
        _delay_ms(1000);
    }
}

void Clear_Buffer(uint8_t line, uint8_t index)
{
	tx_data[line * 16 + index] = ' ';
}

// 텍스트 LCD에 명령을 출력하는 함수 - 단, 비지플래그 체크하지 않음
void LCD_Transmit_Command(char cmd)
{
	cbi(LCD_CON, LCD_RS); // 0번 비트 클리어, RS = 0, 명령
	cbi(LCD_CON, LCD_RW); // 1번 비트 클리어, RW = 0, 쓰기
	sbi(LCD_CON, LCD_E);  // 2번 비트 설정, E = 1
	PORTA = cmd;          // 명령 출력
	_delay_us(1);
	cbi(LCD_CON, LCD_E);  // 명령 쓰기 동작 끝
	_delay_us(1);
}

void LCD_Cursor(char col, char row)
{
	LCD_Transmit_Command(0x80 | (row + col * 0x40));
}


void LCD_Transmit_Data(char data)
{
	sbi(LCD_CON, LCD_RS);
	cbi(LCD_CON, LCD_RW);
	sbi(LCD_CON, LCD_E);
	LCD_DATA = data;
	_delay_us(1);
	cbi(LCD_CON, LCD_E);
	_delay_us(1);
}

void LCD_Transmit_String(char *str)
{
	while(*str)
	LCD_Transmit_Data(*str++);
}
