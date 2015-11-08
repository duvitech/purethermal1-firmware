
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"


#include "usb_task.h"
#include "lepton_task.h"
#include "uart_task.h"
#include "project_config.h"


uint8_t lepton_raw[60*80*2];

extern volatile int restart_frame;
#ifdef USART_DEBUG
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif


PT_THREAD( uart_task(struct pt *pt))
{
	static uint16_t val;
	static int last_frame = 0;
	static int count;
	static int i,j;
	static lepton_buffer *last_buffer;

	PT_BEGIN(pt);

	while (1)
	{
#ifdef THERMAL_DATA_UART 
		 PT_WAIT_UNTIL(pt,get_lepton_frame() != last_frame);
		 last_frame = get_lepton_frame();
		 last_buffer = get_lepton_buffer();

		if ((last_frame % 3) == 0)
		{
			count = 0;
			for (j = 0; j < 60; j++)
			{
				for (i = 2; i < 82; i++)
				{
					val = last_buffer->data[j * 82 + i];

					lepton_raw[count++] = ((val>>8)&0xff);
					lepton_raw[count++] = (val&0xff);
				}
			}
			send_lepton_via_usart(lepton_raw);
		}
#else
		PT_YIELD(pt);
#endif

	}
	PT_END(pt);
}


