

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "main.h"

uint8_t DHT11_I_RH,DHT11_D_RH,DHT11_I_TEMP,DHT11_D_TEMP,DHT11_CheckSum,Calc_CheckSum;
char tx_data[32];
static void Send_String(unsigned char * buf,uint16_t length);
static void Send_Char(unsigned char data);
static void uart1_tx(void);
static void MPU9250_write(char data);
static void I2C_DetectAck(unsigned char step);

void Port_Init(void)
{
    DDRA = 0xFF; //PA0~PA7 Output Direction
    DDRB = 0x20; //PB5 Output Direction
    DDRD = 0x07; //PD0~2 Output Direction
}

void Timer1_Init(void)
{
    TCCR1A = 0x82;  // FAST PWM Mode
    TCCR1B = 0x1B; // FAST PWM MODE(10bit)
    ICR1 = 4999; // 2[MHz] / (1 + 39999) = 50[Hz] -> 20[ms]
    OCR1A = 600; // 20[ms] * 0.1 : 2[ms] : Servo Motor 90[Degree]
}

void LCD_Init(void)
{
	LCD_Transmit_Command(0x38); // Function Set , 2-Line Mode
	_delay_ms(10);	
	LCD_Transmit_Command(0x0C); // Display ON
	_delay_ms(10);
	LCD_Transmit_Command(0x06); // Increment Mode
	_delay_ms(10);
	LCD_Transmit_Command(0x01); // Display Clear
	_delay_ms(10);
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
	uint8_t data = 0;
	uint8_t length = 0;
	uint8_t n1,n10,n100;
	char Humidity;
	char temperature;

    Port_Init();
	Timer1_Init();
    LCD_Init();

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
			length = 0;
			/* Buffer 초기화 */
			for(int i = 0; i<32;i++)
			{
				Clear_Buffer(i);
			}
			/* 1행 첫번째 커서 위치 */
			LCD_Cursor(0,0);
			/* Tx Buffer에 습도 값 넣기 */
			sprintf(&tx_data[length],"Humid : ");
			length = strlen(tx_data);
			n100 = (DHT11_I_RH / 100) % 10;
			n10 = (DHT11_I_RH / 10) % 10;
			n1 = (DHT11_I_RH % 10);
			tx_data[length] = n100 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = n10 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = n1 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = '.'; 
			length = length + 1;			
			tx_data[length] = (DHT11_D_RH % 10) + 0x30; 
			length = length + 1;
			for(int i = 0;i<length;i++)
			{
				/* Tx buffer 출력 */
				LCD_Transmit_Data(tx_data[i]);
			}

			length = 0;
			/* Buffer 초기화 */
			for(int i = 0; i<32;i++)
			{
				Clear_Buffer(i);
			}
			/* 2행 첫번째 커서 위치 */
			LCD_Cursor(1,0);
			/* Tx Buffer에 습도 값 넣기 */
			sprintf(&tx_data[length],"Temp : ");
			length = strlen(tx_data);
			n100 = (DHT11_I_TEMP / 100) % 10;
			n10 = (DHT11_I_TEMP / 10) % 10;
			n1 = (DHT11_I_TEMP % 10);
			tx_data[length] = n100 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = n10 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = n1 + 0x30; // 숫자를 ASCII로 변경
			length = length + 1;
			tx_data[length] = '.'; 
			length = length + 1;			
			tx_data[length] = (DHT11_D_TEMP % 10) + 0x30; 
			length = length + 1;
			for(int i = 0;i<length;i++)
			{
				/* Tx buffer 출력 */
				LCD_Transmit_Data(tx_data[i]);
			}

            if( DHT11_I_RH >= 40)		
            {
                OCR1A = 375; // 20[ms] * 0.075 : 1.5[ms]  ==> Servo Motor 0[Degree]
            }
            else
            {
                OCR1A = 600; // 20[ms] * 0.1 : 2[ms] : Servo Motor 90[Degree]
            }
        }

        _delay_ms(500);
    }
}

void Clear_Buffer(uint8_t index)
{
	tx_data[index] = 0;
}

void LCD_Transmit_Command(char cmd)
{
	cbi(LCD_CON, LCD_RS); // 0번 비트 클리어, RS = 0, 명령
	cbi(LCD_CON, LCD_RW); // 1번 비트 클리어, RW = 0, 쓰기
	_delay_us(10);
	sbi(LCD_CON, LCD_E);  // 2번 비트 설정, E = 1
	PORTA = cmd;          // 명령 출력
	_delay_us(10);
	cbi(LCD_CON, LCD_E);  // 명령 쓰기 동작 끝
	_delay_us(10);
}

void LCD_Cursor(char col, char row)
{
	LCD_Transmit_Command(0x80 | (row + col * 0x40));
}

void LCD_Transmit_Data(char data)
{
	sbi(LCD_CON, LCD_RS);
	cbi(LCD_CON, LCD_RW);
	_delay_us(10);
	sbi(LCD_CON, LCD_E);
	LCD_DATA = data;
	_delay_us(10);
	cbi(LCD_CON, LCD_E);
	_delay_us(10);
}
