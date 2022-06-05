

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "main.h"

uint8_t DHT11_I_RH,DHT11_D_RH,DHT11_I_TEMP,DHT11_D_TEMP,DHT11_CheckSum;

void Port_Init(void)
{
    DDRA = 0xFF; //PA0~PA7 Output Direction
    DDRB = 0x20; //PB5 Output Direction
    DDRD = 0x07; //PD0~2 Output Direction
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

        }
        _delay_ms(3000);
    }
}


// 텍스트 LCD로 부터 상태(명령)를 읽는 함수
unsigned char LCD_rCommand(void)
{
	unsigned char temp=1;
	
	LCD_DATA_DIR = 0X00;
	
	cbi(LCD_CON, LCD_RS); // 0번 비트 클리어, RS = 0, 명령
	sbi(LCD_CON, LCD_RW); // 1번 비트 설정, RW = 1, 읽기
	sbi(LCD_CON, LCD_E);  // 2번 비트 설정, E = 1
	_delay_us(1);
	
	temp = LCD_DATA_IN;      // 명령 읽기
	_delay_us(1);
	
	cbi(LCD_CON, LCD_E);  // 명령 읽기 동작 끝
	
	LCD_DATA_DIR = 0XFF;
	_delay_us(1);
	
	return temp;
}

// 텍스트 LCD의 비지 플래그 상태를 확인하는 함수
char LCD_BusyCheck(unsigned char temp)
{
	return temp & 0x80;
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
	LCD_wBCommand(0x80 | (row + col * 0x40));
}


void LCD_wData(char dat)
{
	while(LCD_BusyCheck(LCD_rCommand()))
	{
		_delay_us(1);
	}
	sbi(LCD_CON, LCD_RS);
	cbi(LCD_CON, LCD_RW);
	sbi(LCD_CON, LCD_E);
	LCD_DATA = dat;
	_delay_us(1);
	cbi(LCD_CON, LCD_E);
	_delay_us(1);
}

void LCD_wString(char *str)
{
	while(*str)
	LCD_wData(*str++);
}
