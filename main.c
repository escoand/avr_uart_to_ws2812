// generic
#include <avr/io.h>
#include <inttypes.h>

// ws2812
#include "light_ws2812-2.2/light_ws2812_AVR/Light_WS2812/light_ws2812.h"

// hardware uart
#ifdef USART_RXC_vect
#define BAUD 9600
#include <avr/interrupt.h>
#include <util/setbaud.h>
static volatile uint8_t inCnt;
static volatile struct cRGB inBuf[180];

// software uart
#else
#include "softuart_gittins_avr/softuart.h"
static uint8_t inCnt;
static struct cRGB inBuf[180];
#endif

void
handleInput (uint8_t c)
{

// end of message
  if (c == 0xFF || c == '\n' || c == '\r'
      || inCnt >= sizeof inBuf / sizeof inBuf[0])
    {
      ws2812_setleds (inBuf, inCnt);
      inCnt = 0;
      sei ();
    }

  // color value
  else if (c & 0x80)
    {
      inBuf[inCnt].r = ((c & 0x30) >> 4) * 0x55;
      inBuf[inCnt].g = ((c & 0x0C) >> 2) * 0x55;
      inBuf[inCnt].b = (c & 0x03) * 0x55;
      inCnt++;
    }
}

#ifdef USART_RXC_vect
// received data by interrupt
ISR (USART_RXC_vect)
{
  UCSRB &= ~(1 << RXCIE);	// disable rx
  handleInput (UDR);
  UCSRB |= (1 << RXCIE);	// enable rx
}
#endif

int
main (void)
{
  inCnt = 0;

  // init uart
#ifdef USART_RXC_vect
  UBRRH = UBRRH_VALUE;
  UBRRL = UBRRL_VALUE;
  UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);	// asynchron 8N1 
  UCSRB |= (1 << RXEN) | (1 << RXCIE);	// enable UART RX and RX interrupt
#else
  softuart_init ();
  softuart_turn_rx_on ();
#endif

  // enable global interrupt
  sei ();

  // main loop
  for (;;)
    {

      // receive data by software uart
#ifndef USART_RXC_vect
      if (softuart_kbhit ())
	{
	  handleInput (softuart_getchar ());
	}
#endif

    }

  return 0;
}
