
 // _ASSERT_ENABLE_ is used for enabling assert, typical for debug purposes
//#define _ASSERT_ENABLE_
#include <string.h>
#include "compiler.h"
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

// set the correct BAUD and F_CPU defines before including setbaud.h
#include "conf_clock.h"			// F_CPU and prescalerp settings
#include "conf_uart.h"

#define rx_size						30	// Command reception buffer size [byte]
#define maxAllowedTemp				120	// [°C]
#define integralErrorLimit			10
#define integralErrorActiveWindow	5	// Error outside defined window will disable integral contribution(to avoid wind-up)
#define allHeaterOff				0b11110000

#define BUFFER_SIZE 255			// UART buffer size

#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_CS   PD4

#include <util/setbaud.h>
#include <avr/interrupt.h>

#include "ring_buffer.h"
#include "delay.h"

// buffers for use with the ring buffer (belong to the UART)
uint8_t out_buffer[BUFFER_SIZE];
uint8_t in_buffer[BUFFER_SIZE];

volatile int pwmCnt_Ch0 = 0;
volatile int pwmCnt_Ch1 = 10;
volatile int pwmCnt_Ch2 = 20;
volatile int pwmCnt_Ch3 = 30;
volatile int pwmCh0 = 0;
volatile int pwmCh1 = 0;
volatile int pwmCh2 = 0;
volatile int pwmCh3 = 0;
volatile bool heaterEnableCh0 = false;
volatile bool heaterEnableCh1 = false;
volatile bool heaterEnableCh2 = false;
volatile bool heaterEnableCh3 = false;

volatile float TempSensor0 = 0;
volatile float TempSensor1 = 0;
volatile float TempSensor2 = 0;
volatile float TempSensor3 = 0;

#define errorHistory	10
const int Dfilter = errorHistory;
volatile int CtrlErrorIdx = 0;
volatile int CtrlErrorHistIdx = 1;

struct channel {
volatile float TempSetPoint;
volatile float P;
volatile float I;
volatile float D;
volatile float Control;
volatile float Control_PID;
volatile float Pgain;
volatile float Igain;
volatile float Dgain;
volatile float TempError[errorHistory + 1];
volatile float tempErrorWin;
volatile int tempSettleCnt;
volatile int tempSettleTime;
volatile int tempStable;	
	};
	
struct channel ch0;

volatile float TempSetPoint0 = 0;
volatile float P_ch0 = 0;
volatile float I_ch0 = 0;
volatile float D_ch0 = 0;
volatile float Control_ch0 = 0;
volatile float Control_ch0_PID = 0;
volatile float Pgain_ch0;
volatile float Igain_ch0;
volatile float Dgain_ch0;
volatile float TempError0[errorHistory + 1];
volatile float tempErrorWinCh0;
volatile int tempSettleCntCh0;
volatile int tempSettleTimeCh0;
volatile int tempStableCh0;

volatile float TempSetPoint1 = 0;
volatile float P_ch1 = 0;
volatile float I_ch1 = 0;
volatile float D_ch1 = 0;
volatile float Control_ch1 = 0;
volatile float Control_ch1_PID = 0;
volatile float Pgain_ch1;
volatile float Igain_ch1;
volatile float Dgain_ch1;
volatile float TempError1[errorHistory + 1];

volatile float TempSetPoint2 = 0;
volatile float P_ch2 = 0;
volatile float I_ch2 = 0;
volatile float D_ch2 = 0;
volatile float Control_ch2 = 0;
volatile float Control_ch2_PID = 0;
volatile float Pgain_ch2;
volatile float Igain_ch2;
volatile float Dgain_ch2;
volatile float TempError2[errorHistory + 1];

volatile float TempSetPoint3 = 0;
volatile float P_ch3 = 0;
volatile float I_ch3 = 0;
volatile float D_ch3 = 0;
volatile float Control_ch3 = 0;
volatile float Control_ch3_PID = 0;
volatile float Pgain_ch3;
volatile float Igain_ch3;
volatile float Dgain_ch3;
volatile float TempError3[errorHistory + 1];

const int eepromAdr_TempSetPoint0	= 0;
const int eepromAdr_PgainCh0		= 4;
const int eepromAdr_IgainCh0		= 8;
const int eepromAdr_DgainCh0		= 12;

const int eepromAdr_TempSetPoint1	= 16;
const int eepromAdr_PgainCh1		= 20;
const int eepromAdr_IgainCh1		= 24;
const int eepromAdr_DgainCh1		= 28;

const int eepromAdr_TempSetPoint2	= 32;
const int eepromAdr_PgainCh2		= 36;
const int eepromAdr_IgainCh2		= 40;
const int eepromAdr_DgainCh2		= 44;

const int eepromAdr_TempSetPoint3	= 48;
const int eepromAdr_PgainCh3		= 52;
const int eepromAdr_IgainCh3		= 56;
const int eepromAdr_DgainCh3		= 60;

const char statusLed = 0b00001000;

char requestCmd[3];
char setCmd[3];
char rx_string[30];
char crc[5];

// Prototypes
static inline void SetHeaterOutputON(uint8_t ch);
static inline float ReadTempSensor(uint8_t ch);
static inline void ReadParameter(int id);
static inline void SetParameter(int id);
static inline void ParamParse(char *stringToParse, char *paramFloat);
void ftoa(float num, char *str);
static inline int GenCrc16(char c[], int nByte);
static inline bool CrcCompare(char *crc_in, char *crc_calc);
static inline void ReadParmEEPROM(void);
static inline void WriteParamToEEPROM(void);

struct ring_buffer ring_buffer_out;		//! ring buffer to use for the UART transmission
struct ring_buffer ring_buffer_in;		//! ring buffer to use for the UART reception

/**
 * \brief UART data register empty interrupt handler
 *
 * This handler is called each time the UART data register is available for
 * sending data.
 */
ISR(UART0_DATA_EMPTY_IRQ)
{
	// if there is data in the ring buffer, fetch it and send it
	if (!ring_buffer_is_empty(&ring_buffer_out))
	{
		UDR0 = ring_buffer_get(&ring_buffer_out);
	}
	else 
	{
		// no more data to send, turn off data ready interrupt
		UCSR0B &= ~(1 << UDRIE0);
	}
}

/**
 * \brief Data RX interrupt handler
 *
 * This is the handler for UART receive data
 */
ISR(UART0_RX_IRQ)
{
	ring_buffer_put(&ring_buffer_in, UDR0);
}

/**
 * \brief Initialize the UART with correct baud rate settings
 *
 * This function will initialize the UART baud rate registers with the correct
 * values using the AVR libc setbaud utility. In addition set the UART to
 * 8-bit, 1 stop and no parity.
 */
static void uart_init(void)
{
#if defined UBRR0H
	// get the values from the setbaud tool
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

#if USE_2X
	UCSR0A |= (1 << U2X0);
#endif

	// enable RX and TX and set interrupts on rx complete
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// 8-bit, 1 stop bit, no parity, asynchronous UART
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << USBS0) |
			(0 << UPM01) | (0 << UPM00) | (0 << UMSEL01) |
			(0 << UMSEL00);

	// initialize the in and out buffer for the UART
	ring_buffer_out = ring_buffer_init(out_buffer, BUFFER_SIZE);
	ring_buffer_in = ring_buffer_init(in_buffer, BUFFER_SIZE);
}

/**
 * \brief Function for putting a char in the UART buffer
 *
 * \param data the data to add to the UART buffer and send
 *
 */
static inline void uart_putchar(uint8_t data)
{
	// Disable interrupts to get exclusive access to ring_buffer_out.
	cli();
	if (ring_buffer_is_empty(&ring_buffer_out)) 
	{
		// First data in buffer, enable data ready interrupt
		UCSR0B |=  (1 << UDRIE0);
	}
	// Put data in buffer
	ring_buffer_put(&ring_buffer_out, data);

	// Re-enable interrupts
	sei();
}

/**
 * \brief Function for getting a char from the UART receive buffer
 *
 * \retval Next data byte in receive buffer
 */
static inline uint8_t uart_getchar(void)
{
	return ring_buffer_get(&ring_buffer_in);
}

/**
 * \brief Function to check if we have a char waiting in the UART receive buffer
 *
 * \retval true if data is waiting
 * \retval false if no data is waiting
 */
static inline bool uart_char_waiting(void)
{
	return !ring_buffer_is_empty(&ring_buffer_in);
}

static inline void TimerInit(void)
{
	// Timer0 PWM output
	TIMSK0 = _BV(OCIE0A);				// Enable Interrupt TimerCounter0 Compare Match A (TIMER0_COMPA_vect)
	TCCR0A = _BV(WGM01);				// Mode = CTC (Clear Timer on Compare)
	TCCR0B = _BV(CS02) | _BV(CS00);		// 16MHz/1024, 0.000064 seconds per tick
	OCR0A = 16;							// TIMER0_COMPA_vect will be triggered each 0.000064 x 16 = 0.001024 s (976.5 Hz)

	// Timer2 PID controller	
	TIMSK2 = _BV(OCIE2A);				// Enable Interrupt TimerCounter2 Compare Match A (TIMER2_COMPA_vect)
	TCCR2A = _BV(WGM21);				// Mode = CTC (Clear Timer on Compare)
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);		// 16MHz/1024, 0.000064 seconds per tick
	OCR2A = 255;						// TIMER2_COMPA_vect will be triggered each 0.000064 x 255 = 0.01632 s (61.3 Hz)
	
	sei();
}

ISR(TIMER0_COMPA_vect)	// PWM output
{
	pwmCnt_Ch0++;
	if(pwmCnt_Ch0 < pwmCh0 )
	{
		SetHeaterOutputON(0);
	}
	else
	{
		PORTC &= 0b11111110;
	}
	if(pwmCnt_Ch0 >= 100)
	{
		pwmCnt_Ch0 = 0;
	}
	
	pwmCnt_Ch1++;
	if(pwmCnt_Ch1 < pwmCh1 )
	{
		SetHeaterOutputON(1);
	}
	else
	{
		PORTC &= 0b11111101;
	}
	if(pwmCnt_Ch1 >= 100)
	{
		pwmCnt_Ch1 = 0;
	}
	
	pwmCnt_Ch2++;
	if(pwmCnt_Ch2 < pwmCh2 )
	{
		SetHeaterOutputON(2);
	}
	else
	{
		PORTC &= 0b11111011;
	}
	if(pwmCnt_Ch2 >= 100)
	{
		pwmCnt_Ch2 = 0;
	}
	
	pwmCnt_Ch3++;
	if(pwmCnt_Ch3 < pwmCh3 )
	{
		SetHeaterOutputON(3);
	}
	else
	{
		PORTC &= 0b11110111;
	}
	if(pwmCnt_Ch3 >= 100)
	{
		pwmCnt_Ch3 = 0;
	}

}

ISR(TIMER2_COMPA_vect)	// PID Controller
{
	sei();	// Enable nested interrupt
	PORTD |= statusLed;
	
	TempSensor0 = ReadTempSensor(1);
	if(TempSensor0 > (float)maxAllowedTemp)					// Max temperature limit
	{
		heaterEnableCh0 = false;
		PORTC &= allHeaterOff;
	}

	TempError0[CtrlErrorIdx] = TempSetPoint0 - TempSensor0;			// Control error
	
	P_ch0 = TempError0[CtrlErrorIdx] * Pgain_ch0;					// P error
	I_ch0 = I_ch0 + (TempError0[CtrlErrorIdx] * Igain_ch0);			// I error
	if(I_ch0 > integralErrorLimit)
	{
	I_ch0 = integralErrorLimit;
	}
	if(I_ch0 < -integralErrorLimit)
	{
	I_ch0 = -integralErrorLimit;
	}
	
	D_ch0 = (TempError0[CtrlErrorIdx] - TempError0[CtrlErrorHistIdx]) * Dgain_ch0; // D error
	
	Control_ch0_PID = P_ch0 + I_ch0 + D_ch0;
	Control_ch0 = Control_ch0_PID + TempSetPoint0 * (float)0.21 - 5;
	pwmCh0 = (round(Control_ch0));
	
	if(abs(TempError0[0]) > integralErrorActiveWindow)
	{
	I_ch0 = 0;									// Avoid integral wind-up
	}
	
	if(abs(TempError0[0]) <= tempErrorWinCh0)
	{
		tempSettleCntCh0 ++;
	}
	else
	{
		tempSettleCntCh0 = 0;
	}
	if(tempSettleCntCh0 >= tempSettleTimeCh0)
	{
		tempStableCh0 = 1;
	}
	else
	{
		tempStableCh0 = 0;
	}
	
	//*** Channel 1 ***
	TempSensor1 = ReadTempSensor(2);
		
	if(TempSensor1 > (float)maxAllowedTemp)					// Max temperature limit
	{
		heaterEnableCh1 = false;
		PORTC &= allHeaterOff;
	}

	TempError1[CtrlErrorIdx] = TempSetPoint1 - TempSensor1;			// Control error
	
	P_ch1 = TempError1[CtrlErrorIdx] * Pgain_ch1;					// P error
	I_ch1 = I_ch1 + (TempError1[CtrlErrorIdx] * Igain_ch1);			// I error
	if(I_ch1 > integralErrorLimit)
	{
		I_ch1 = integralErrorLimit;
	}
	if(I_ch1 < -integralErrorLimit)
	{
		I_ch1 = -integralErrorLimit;
	}
	
	D_ch1 = (TempError1[CtrlErrorIdx] - TempError1[CtrlErrorHistIdx]) * Dgain_ch1; // D error
	
	Control_ch1_PID = P_ch1 + I_ch1 + D_ch1;
	Control_ch1 = Control_ch1_PID + TempSetPoint1 * (float)0.21 - 5;
	pwmCh1 = (round(Control_ch1));
	
	if(abs(TempError1[0]) > integralErrorActiveWindow)
	{
		I_ch1 = 0;									// Avoid integral wind-up
	}
	
	// *** Channel 2 ***
	TempSensor2 = ReadTempSensor(3);
	
	if(TempSensor2 > (float)maxAllowedTemp)					// Max temperature limit
	{
		heaterEnableCh2 = false;
		PORTC &= allHeaterOff;
	}
	
	TempError2[CtrlErrorIdx] = TempSetPoint2 - TempSensor2;			// Control error
		
	P_ch2 = TempError2[CtrlErrorIdx] * Pgain_ch2;					// P error
	I_ch2 = I_ch2 + (TempError2[CtrlErrorIdx] * Igain_ch2);			// I error
	if(I_ch2 > integralErrorLimit)
	{
		I_ch2 = integralErrorLimit;
	}
	if(I_ch2 < -integralErrorLimit)
	{
		I_ch2 = -integralErrorLimit;
	}
		
	D_ch2 = (TempError2[CtrlErrorIdx] - TempError2[CtrlErrorHistIdx]) * Dgain_ch2; // D error
		
	Control_ch2_PID = P_ch2 + I_ch2 + D_ch2;
	Control_ch2 = Control_ch2_PID + TempSetPoint2 * (float)0.21 - 5;
	pwmCh2 = (round(Control_ch2));
		
	if(abs(TempError2[0]) > integralErrorActiveWindow)
	{
		I_ch2 = 0;									// Avoid integral wind-up
	}

	// *** Channel 3 ***
	TempSensor3 = ReadTempSensor(4);
	
	if(TempSensor3 > (float)maxAllowedTemp)					// Max temperature limit
	{
		heaterEnableCh3 = false;
		PORTC &= allHeaterOff;
	}
	
	TempError3[CtrlErrorIdx] = TempSetPoint3 - TempSensor3;			// Control error
	
	P_ch3 = TempError3[CtrlErrorIdx] * Pgain_ch3;					// P error
	I_ch3 = I_ch3 + (TempError3[CtrlErrorIdx] * Igain_ch3);			// I error
	if(I_ch3 > integralErrorLimit)
	{
		I_ch3 = integralErrorLimit;
	}
	if(I_ch3 < -integralErrorLimit)
	{
		I_ch3 = -integralErrorLimit;
	}
	
	D_ch3 = (TempError3[CtrlErrorIdx] - TempError3[CtrlErrorHistIdx]) * Dgain_ch3; // D error
	
	Control_ch3_PID = P_ch3 + I_ch3 + D_ch3;
	Control_ch3 = Control_ch3_PID + TempSetPoint3 * (float)0.21 - 5;
	pwmCh3 = (round(Control_ch3));
	
	if(abs(TempError3[0]) > integralErrorActiveWindow)
	{
		I_ch3 = 0;									// Avoid integral wind-up
	}	

	CtrlErrorIdx ++;							// Control error ring buffer index
	if(CtrlErrorIdx > errorHistory)
	{
		CtrlErrorIdx = 0;
	}
	CtrlErrorHistIdx = CtrlErrorIdx + 1;
	if(CtrlErrorHistIdx > errorHistory)
	{
		CtrlErrorHistIdx = 0;
	}		

	PORTD &= 0b11110111;						// Turn off LED on PCB (irq call freq: 16MHz / 1024 / 255 = 61.3 Hz )
}

static inline void SetHeaterOutputON(uint8_t ch)
{
	switch (ch)
	{
		case 0:
			if(heaterEnableCh0)
			{
				PORTC |= 0b00000001;
			}
			break;
	
		case 1:
			if(heaterEnableCh1)
			{
				PORTC |= 0b00000010;	
			}
			break;

		case 2:
			if(heaterEnableCh2)
			{
				PORTC |= 0b00000100;
			}
			break;

		case 3:
			if(heaterEnableCh3)
			{
				PORTC |= 0b00001000;	
			}
			break;				
	}
}

/************************************************
SPI INITIALISATION
*************************************************/
static inline void SPIinit(void)
{
	SPI_DDR &= 0b11101111;							// PB4 = Input = MISO
	SPI_DDR = (1<<PORTB3)|(1<<PORTB5|(1<<PORTB2));	// Set MOSI, SCK and SS as output
	
	
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<CPHA)|(1<<CPOL); // Enable SPI, Master, set clock rate fck/16
}

/************************************************
SPIread
*************************************************/
static inline unsigned char SPIread( unsigned char byteword, unsigned char ch)
{
	switch (ch)
	{
		case 1:
		PORTD &= 0b11101111;	// Set CS1 low
		break;
		
		case 2:
		PORTD &= 0b11011111;	// Set CS2 low
		break;
				
		case 3:
		PORTD &= 0b10111111;	// Set CS3 low
		break;
						
		case 4:
		PORTD &= 0b01111111;	// Set CS4 low
		break;
	}
	//delay_us(1);
	SPDR = byteword;			// put the byteword into data register
	while(!(SPSR & (1<<SPIF)));	// Wait for transmission complete
	byteword=SPDR;				//just for clearing SPIF
	
	SPDR = 0x00;			// put the byteword into data register
	while(!(SPSR & (1<<SPIF)));	// Wait for transmission complete
	byteword=SPDR;
	
	PORTD |= 0b11110000;	// Set all CS High
	return byteword;
}

/************************************************
MAX31865init
*************************************************/
static inline void MaxRTDinit( unsigned char dataWrite, unsigned char ch)
{
	unsigned char dummy;

	switch (ch)
	{
		case 1:
		PORTD &= 0b11101111;	// Set CS1 low
		break;
		
		case 2:
		PORTD &= 0b11011111;	// Set CS2 low
		break;
		
		case 3:
		PORTD &= 0b10111111;	// Set CS3 low
		break;
		
		case 4:
		PORTD &= 0b01111111;	// Set CS4 low
		break;
	}
	SPDR = 0x80;				// Configuration
	while(!(SPSR & (1<<SPIF)));	// Wait for transmission complete
	dummy = SPDR;				//just for clearing SPIF
	SPDR = dataWrite;
	while(!(SPSR & (1<<SPIF)));	// Wait for transmission complete
	dummy = SPDR;
	
	PORTD |= 0b11110000;		// Set all CS High
}

static inline void printStatus (char msg_string[])	// Send string on UART
{
	int cnt, i; 
	uint16_t crcXmodem;
	char tx_string[40];
	tx_string[0] = '!';
	
	for(cnt = 1; cnt < 4; cnt ++)
	{
		tx_string[cnt] = rx_string[cnt];			// Add echo sequence number
	} 
	for(cnt = 0; cnt < strlen(msg_string)+3; cnt++)	// Add message
	{
		tx_string[cnt+4] = msg_string[cnt];
	}

	crcXmodem = GenCrc16(tx_string, cnt+1);
	itoa(crcXmodem, crc, 16);
	strupr(crc);									// Convert crc to upper case
	if(crcXmodem < 0x10)
	{
		tx_string[cnt+4] = crc[0];
		tx_string[cnt+3] = '0';
		tx_string[cnt+2] = '0';
		tx_string[cnt+1] = '0';
	}
	if(crcXmodem < 0x100 && crcXmodem >= 0x10)
	{
		tx_string[cnt+4] = crc[1];
		tx_string[cnt+3] = crc[0];
		tx_string[cnt+2] = '0';
		tx_string[cnt+1] = '0';		
	}	
	if(crcXmodem < 0x1000 && crcXmodem >= 0x0100)
	{
		tx_string[cnt+4] = crc[2];
		tx_string[cnt+3] = crc[1];
		tx_string[cnt+2] = crc[0];
		tx_string[cnt+1] = '0';		
	}
	if(crcXmodem >= 0x1000)
	{
		tx_string[cnt+1] = crc[0];					// Add crc	
		tx_string[cnt+2] = crc[1];
		tx_string[cnt+3] = crc[2];
		tx_string[cnt+4] = crc[3];
	}
	
	tx_string[cnt+5] = 0x0D;						// End with <CR>
	
	for(i = 0; i < cnt+6; i++)						// Send message on UART
	{
		uart_putchar(tx_string[i]);
	}	
}

static inline void ReadInit(uint8_t ch)
{
	char data[1];
	data[0] = SPIread(0x00, ch);
	printStatus("IntiRead: ");
	printStatus(data);
}

void ftoa(float num, char *str)
{
	int intpart = num;
	int intdecimal;
	int i;
	float decimal_part;
	char decimal[20];
	int PRECISION = 3;						// Decimal precision

	memset(str, 0x0, 20);
	itoa(num, str, 10);

	strcat(str, ".");

	decimal_part = num - intpart;
	intdecimal = decimal_part * 1000;		// If decimal precision is 3 then 1000, 2 -> 100, 1 -> 10 ...;

	if(intdecimal < 0)
	{
		intdecimal = -intdecimal;
	}
	itoa(intdecimal, decimal, 10);
	for(i =0;i < (PRECISION - strlen(decimal));i++)
	{
		strcat(str, "0");
	}
	strcat(str, decimal);
}

static inline float ReadTempSensor(uint8_t ch)
{
	int adc_code = 0x0000;
	uint8_t u8_msb = 0;
	uint8_t u8_lsb = 0;
	
	u8_msb = SPIread(0x01, ch);
	u8_lsb = SPIread(0x02, ch);
	adc_code = (((u8_msb << 8) | u8_lsb) >> 1);
	return ((float)adc_code / 32.0 - 256.0);
}

static inline void ReadFault(uint8_t ch)
{
	char data[1];
	data[0] = SPIread(0x07, ch);
	printStatus("Fault: ");
	printStatus(data);
}

static inline void MAX31865initAuto(uint8_t ch)
{
	MaxRTDinit(0xC3, ch);
}

static inline void StepperCV(int step, bool direction)
{
	volatile int i;
	switch (direction)
	{
		case true:
		PORTB |= 0b00000100;
		break;
		
		case false:
		PORTB &= 0b11111011;
		break;
	}
	for (i = 0; i < step; i++)
	{
		PORTB ^= 0b00000010;
		delay_us(10);
	}
}

static inline void ReadParmEEPROM()
{
	TempSetPoint0 = eeprom_read_float((float*)eepromAdr_TempSetPoint0);
	Pgain_ch0 = eeprom_read_float((float*)eepromAdr_PgainCh0);
	Igain_ch0 = eeprom_read_float((float*)eepromAdr_IgainCh0);
	Dgain_ch0 = eeprom_read_float((float*)eepromAdr_DgainCh0);
	
	TempSetPoint1 = eeprom_read_float((float*)eepromAdr_TempSetPoint1);
	Pgain_ch1 = eeprom_read_float((float*)eepromAdr_PgainCh1);
	Igain_ch1 = eeprom_read_float((float*)eepromAdr_IgainCh1);
	Dgain_ch1 = eeprom_read_float((float*)eepromAdr_DgainCh1);
	
	TempSetPoint2 = eeprom_read_float((float*)eepromAdr_TempSetPoint2);
	Pgain_ch2 = eeprom_read_float((float*)eepromAdr_PgainCh2);
	Igain_ch2 = eeprom_read_float((float*)eepromAdr_IgainCh2);
	Dgain_ch2 = eeprom_read_float((float*)eepromAdr_DgainCh2);
	
	TempSetPoint3 = eeprom_read_float((float*)eepromAdr_TempSetPoint3);
	Pgain_ch3 = eeprom_read_float((float*)eepromAdr_PgainCh3);
	Igain_ch3 = eeprom_read_float((float*)eepromAdr_IgainCh3);
	Dgain_ch3 = eeprom_read_float((float*)eepromAdr_DgainCh3);			
}

static inline void WriteParamToEEPROM()
{
	eeprom_write_float( (float*)eepromAdr_TempSetPoint0, TempSetPoint0 );
	eeprom_write_float( (float*)eepromAdr_PgainCh0, Pgain_ch0 );
	eeprom_write_float( (float*)eepromAdr_IgainCh0, Igain_ch0 );
	eeprom_write_float( (float*)eepromAdr_DgainCh0, Dgain_ch0 );
	
	eeprom_write_float( (float*)eepromAdr_TempSetPoint1, TempSetPoint1 );
	eeprom_write_float( (float*)eepromAdr_PgainCh1, Pgain_ch1 );
	eeprom_write_float( (float*)eepromAdr_IgainCh1, Igain_ch1 );
	eeprom_write_float( (float*)eepromAdr_DgainCh1, Dgain_ch1 );
	
	eeprom_write_float( (float*)eepromAdr_TempSetPoint2, TempSetPoint2 );
	eeprom_write_float( (float*)eepromAdr_PgainCh2, Pgain_ch2 );
	eeprom_write_float( (float*)eepromAdr_IgainCh2, Igain_ch2 );
	eeprom_write_float( (float*)eepromAdr_DgainCh2, Dgain_ch2 );
	
	eeprom_write_float( (float*)eepromAdr_TempSetPoint3, TempSetPoint3 );
	eeprom_write_float( (float*)eepromAdr_PgainCh3, Pgain_ch3 );
	eeprom_write_float( (float*)eepromAdr_IgainCh3, Igain_ch3 );
	eeprom_write_float( (float*)eepromAdr_DgainCh3, Dgain_ch3 );			
}

int main(void)
{
	char requestID[4];
	char s_string[20] = "C2CA SA ADP 1.0";
	char *pos;
	char inCrC_string[5];
	int i;
	int var;
	
	cli();
	uart_init();
	sei();
	SPIinit();

	pwmCh0 = 1;
	pwmCh1 = 1;
	pwmCh2 = 1;
	pwmCh3 = 1;

	DDRD = 0b11111000;			// 1 = output 0 = input
	DDRC = 0b00001111;
	DDRB |= (1<<PORTB1);		// Set stepper Dir to output
	
	PORTD |=(1<<PORTD7)|(1<<PORTD6)|(1<<PORTD5)|(1<<PORTD4);	// Set initially all CS high
	
	ReadParmEEPROM();
	MAX31865initAuto(1);
	MAX31865initAuto(2);
	MAX31865initAuto(3);
	MAX31865initAuto(4);
	TimerInit();	
	
	/***********************************************************
	*	Msg example:
	*	PC:		#12A?VR104
	*	ATMega:	!12A70.0006DCA
	***********************************************************/
	while(1)
	{
		memset(rx_string, 0x00, rx_size);			// Purge rx_string

		for(i = 0; i < rx_size; i++)
		{
			while(!uart_char_waiting());
			rx_string[i] = uart_getchar();			// Collect the message
			if(rx_string[i] == 0x0D)				// Terminate acquisition if <CR> received
			{
				break;
			}	
		}

		if(rx_string[0] == '#')
		{
			pos = NULL;
			pos = strchr(rx_string, '?');
			if(pos != NULL)							
			{
				requestCmd[0] = *(pos + 1);			// Parse request command	
				requestCmd[1] = *(pos + 2);

				if(strcmp(requestCmd, "IF") == 0)
				{
					crc[0] = *(pos + 3);			// Parse incoming crc
					crc[1] = *(pos + 4);
					crc[2] = *(pos + 5);
					crc[3] = *(pos + 6);

					itoa(GenCrc16(rx_string, (int)strnlen(rx_string, rx_size)-5), inCrC_string, 16);
					if(CrcCompare(crc, inCrC_string) == true)
					{
						printStatus(s_string);	
					}
					else
					{
						printStatus("CRC error");
						//printStatus(rx_string);
					}
				}
					
				if(strcmp(requestCmd, "VR") == 0)
				{
					requestID[0] = *(pos + 3);		// Parse request ID
					requestID[1] = *(pos + 4);
					requestID[2] = *(pos + 5);
					crc[0] = *(pos + 6);			// Parse incoming crc
					crc[1] = *(pos + 7);
					crc[2] = *(pos + 8);
					crc[3] = *(pos + 9);
					
					itoa(GenCrc16(rx_string, (int)strnlen(rx_string, rx_size)-5), inCrC_string, 16);
					if(CrcCompare(crc, inCrC_string) == true)
					{					
						var = atol(requestID);
						ReadParameter(var);
					}
					else
					{
						printStatus("CRC error");
						//printStatus(rx_string);
					}
				}
			}	
			
			pos = NULL;
			pos = strchr(rx_string, '&');
			if(pos != NULL)
			{
				setCmd[0] = *(pos + 1);					// Parse set command
				setCmd[1] = *(pos + 2);
				requestID[0] = *(pos + 3);				// Parse request ID
				requestID[1] = *(pos + 4);
				requestID[2] = *(pos + 5);				
				
				if(strcmp(setCmd, "VS") == 0)
				{
					pos = strchr(rx_string, 0x0D);		// Find end of message
					crc[0] = *(pos - 4);				// Parse incoming crc
					crc[1] = *(pos - 3);
					crc[2] = *(pos - 2);
					crc[3] = *(pos - 1);
					
					itoa(GenCrc16(rx_string, (int)strnlen(rx_string, rx_size)-5), inCrC_string, 16);
					if(CrcCompare(crc, inCrC_string) == true)
					{
						var = atol(requestID);
						SetParameter(atol(requestID));
					}
					else
					{
						printStatus("CRC error");
						//printStatus(rx_string);
					} 
				}
			}
		}
	}

}

static inline void ReadParameter(int id)
{
	char tx_string[20];

	switch(id)
	{
		// *** Channel 0 ***
		case 100:	// Send sensor 1 temperature		
		ftoa(TempSensor0, tx_string);
		printStatus(tx_string);
		break;
		
		case 101:	// Send P part of PID-controller 1
		ftoa(P_ch0, tx_string);
		printStatus(tx_string);
		break;
		
		case 102:	// Send I part of PID-controller 1
		ftoa(I_ch0, tx_string);
		printStatus(tx_string);
		break;
				
		case 103:	// Send D part of PID-controller 1
		ftoa(D_ch0, tx_string);
		printStatus(tx_string);
		break;
		
		case 104:	// Send TempSetPoint0
		ftoa(TempSetPoint0, tx_string);
		printStatus(tx_string);
		break;
		
		case 105:	// Send Pgain ch0
		ftoa(Pgain_ch0, tx_string);
		printStatus(tx_string);
		break;								
		
		case 106:	// Send Igain ch0
		ftoa(Igain_ch0, tx_string);
		printStatus(tx_string);
		break;
		
		case 107:	// Send Dgain ch0
		ftoa(Dgain_ch0, tx_string);
		printStatus(tx_string);
		break;
		
		// *** Channel 1 ***
		case 200:	// Send sensor 1 temperature
		ftoa(TempSensor1, tx_string);
		printStatus(tx_string);
		break;
		
		case 201:	// Send P part of PID-controller 1
		ftoa(P_ch1, tx_string);
		printStatus(tx_string);
		break;
		
		case 202:	// Send I part of PID-controller 1
		ftoa(I_ch1, tx_string);
		printStatus(tx_string);
		break;
				
		case 203:	// Send D part of PID-controller 1
		ftoa(D_ch1, tx_string);
		printStatus(tx_string);
		break;
		
		case 204:	// Send TempSetPoint1
		ftoa(TempSetPoint1, tx_string);
		printStatus(tx_string);
		break;
		
		case 205:	// Send Pgain ch1
		ftoa(Pgain_ch1, tx_string);
		printStatus(tx_string);
		break;
		
		case 206:	// Send Igain ch1
		ftoa(Igain_ch1, tx_string);
		printStatus(tx_string);
		break;
		
		case 207:	// Send Dgain ch1
		ftoa(Dgain_ch1, tx_string);
		printStatus(tx_string);
		break;

		// *** Channel 2 ***		
		case 300:	// Send sensor 2 temperature
		ftoa(TempSensor2, tx_string);
		printStatus(tx_string);
		break;
		
		case 301:	// Send P part of PID-controller 2
		ftoa(P_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		case 302:	// Send I part of PID-controller 2
		ftoa(I_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		case 303:	// Send D part of PID-controller 2
		ftoa(D_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		case 304:	// Send TempSetPoint2
		ftoa(TempSetPoint2, tx_string);
		printStatus(tx_string);
		break;
		
		case 305:	// Send Pgain ch2
		ftoa(Pgain_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		case 306:	// Send Igain ch2
		ftoa(Igain_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		case 307:	// Send Dgain ch2
		ftoa(Dgain_ch2, tx_string);
		printStatus(tx_string);
		break;
		
		// *** Channel 3 ***
		case 400:	// Send sensor 3 temperature
		ftoa(TempSensor3, tx_string);
		printStatus(tx_string);
		break;
		
		case 401:	// Send P part of PID-controller 3
		ftoa(P_ch3, tx_string);
		printStatus(tx_string);
		break;
		
		case 402:	// Send I part of PID-controller 3
		ftoa(I_ch3, tx_string);
		printStatus(tx_string);
		break;
		
		case 403:	// Send D part of PID-controller 3
		ftoa(D_ch3, tx_string);
		printStatus(tx_string);
		break;
		
		case 404:	// Send TempSetPoint3
		ftoa(TempSetPoint3, tx_string);
		printStatus(tx_string);
		break;
		
		case 405:	// Send Pgain ch3
		ftoa(Pgain_ch3, tx_string);
		printStatus(tx_string);
		break;
		
		case 406:	// Send Igain ch3
		ftoa(Igain_ch3, tx_string);
		printStatus(tx_string);
		break;
		
		case 407:	// Send Dgain ch3
		ftoa(Dgain_ch3, tx_string);
		printStatus(tx_string);
		break;						
		
		default:
		printStatus("VRerror");
	}
}

static inline void SetParameter(int id)
{
	char param[20];
	
	switch(id)
	{
		// *** Channel 0 ***
		case 150:	// TempSetPoint0
		ParamParse(rx_string, param);
		TempSetPoint0 = atof(param);
		printStatus("");
		break;
		
		case 151:	// SetPgainCh0
		ParamParse(rx_string, param);
		Pgain_ch0 = atof(param);
		printStatus("");
		break;
		
		case 152:	// SetIgainCh0
		ParamParse(rx_string, param);
		Igain_ch0 = atof(param);
		printStatus("");
		break;
		
		case 153:	// SetDgain_ch0
		ParamParse(rx_string, param);
		Dgain_ch0 = atof(param);
		printStatus("");
		break;	
		
		case 154:	// Set heater on/off ch0
		if(rx_string[10] == '1')
		{
			heaterEnableCh0 = true;
			printStatus("ON");
		}
		else if(rx_string[10] == '0')
		{
			heaterEnableCh0 = false;
			printStatus("OFF");
		}
		break;	
		
		// *** Channel 1 ***
		case 250:	// TempSetPoint1
		ParamParse(rx_string, param);
		TempSetPoint1 = atof(param);
		printStatus("");
		break;
		
		case 251:	// SetPgainCh1
		ParamParse(rx_string, param);
		Pgain_ch1 = atof(param);
		printStatus("");
		break;
		
		case 252:	// SetIgainCh1
		ParamParse(rx_string, param);
		Igain_ch1 = atof(param);
		printStatus("");
		break;
		
		case 253:	// SetDgain_ch1
		ParamParse(rx_string, param);
		Dgain_ch1 = atof(param);
		printStatus("");
		break;
		
		case 254:	// Set heater on/off ch1
		if(rx_string[10] == '1')
		{
			heaterEnableCh1 = true;
			printStatus("ON");
		}
		else if(rx_string[10] == '0')
		{
			heaterEnableCh1 = false;
			printStatus("OFF");
		}
		break;
		
		// *** Channel 2 ***
		case 350:	// TempSetPoint2
		ParamParse(rx_string, param);
		TempSetPoint2 = atof(param);
		printStatus("");
		break;
		
		case 351:	// SetPgainCh2
		ParamParse(rx_string, param);
		Pgain_ch2 = atof(param);
		printStatus("");
		break;
		
		case 352:	// SetIgainCh2
		ParamParse(rx_string, param);
		Igain_ch2 = atof(param);
		printStatus("");
		break;
		
		case 353:	// SetDgain_ch2
		ParamParse(rx_string, param);
		Dgain_ch2 = atof(param);
		printStatus("");
		break;
		
		case 354:	// Set heater on/off ch2
		if(rx_string[10] == '1')
		{
			heaterEnableCh2 = true;
			printStatus("ON");
		}
		else if(rx_string[10] == '0')
		{
			heaterEnableCh2 = false;
			printStatus("OFF");
		}
		break;
		
		// *** Channel 4 ***
		case 450:	// TempSetPoint3
		ParamParse(rx_string, param);
		TempSetPoint3 = atof(param);
		printStatus("");
		break;
		
		case 451:	// SetPgainCh3
		ParamParse(rx_string, param);
		Pgain_ch3 = atof(param);
		printStatus("");
		break;
		
		case 452:	// SetIgainCh3
		ParamParse(rx_string, param);
		Igain_ch3 = atof(param);
		printStatus("");
		break;
		
		case 453:	// SetDgain_ch3
		ParamParse(rx_string, param);
		Dgain_ch3 = atof(param);
		printStatus("");
		break;
		
		case 454:	// Set heater on/off ch3
		if(rx_string[10] == '1')
		{
			heaterEnableCh3 = true;
			printStatus("ON");
		}
		else if(rx_string[10] == '0')
		{
			heaterEnableCh3 = false;
			printStatus("OFF");
		}
		break;						 

		case 500: // Store param to EEPROM
		WriteParamToEEPROM();
		printStatus("");
		break;
		
		default:
		printStatus("VSerror");
	}
}

static inline void ParamParse(char *stringToParse, char *param)
{
	int s_len = strnlen(stringToParse, rx_size) - 5;
	int idx = 0;
		
	for(int i = 10; i < s_len; i++)
	{
		param[idx] = stringToParse[i];
		idx ++;
	}	
}

static inline int GenCrc16(char c[], int nByte)	// CRC-CCITT (XModem)
{
	int Polynominal = 0x1021;
	int InitValue = 0x0;

	int i, j, index = 0;
	int CRC = InitValue;
	int Remainder, tmp, int_c;
	for (i = 0; i < nByte; i++)
	{
		int_c = (int)(0x00ff & (int)c[index]);
		tmp = (int)((CRC >> 8) ^ int_c);
		Remainder = (int)(tmp << 8);
		for (j = 0; j < 8; j++)
		{

			if ((Remainder & 0x8000) != 0)
			{
				Remainder = (int)((Remainder << 1) ^ Polynominal);
			}
			else
			{
				Remainder = (int)(Remainder << 1);
			}
		}
		CRC = (int)((CRC << 8) ^ Remainder);
		index++;
	}
	return CRC;
}

static inline bool CrcCompare(char *crc_in, char *crc_calc)
{
	strupr(crc_in);
	strupr(crc_calc);
	
	if(crc_in[0] == '0' && crc_in[1] == '0' && crc_in[2] == '0')
	{
			if(crc_in[3] == crc_calc[0])
			{
				return true;
			}
			else
			{
				return false;
			}
	}

	if(crc_in[0] == '0' && crc_in[1] == '0')
	{
		if((crc_in[2] == crc_calc[0]) && (crc_in[3] == crc_calc[1]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	if(crc_in[0] == '0')
	{
		if((crc_in[1] == crc_calc[0]) && (crc_in[2] == crc_calc[1]) &&
		(crc_in[3] == crc_calc[2]))
		{
			return true;
		}
		else
		{
			return false;
		}
	}	
	else
	{
		if((crc_in[0] == crc_calc[0]) && (crc_in[1] == crc_calc[1]) &&
			(crc_in[2] == crc_calc[2]) && (crc_in[3] == crc_calc[3]))
			{
				return true;
			}
			else
			{
				return false;
			}
	}
}
