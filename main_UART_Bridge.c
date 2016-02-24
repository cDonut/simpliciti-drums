/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *----------------------------------------------------------------------------
 */
/******************************************************************************************

  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
#define INDICATOR_TIME_OUT 250;

#include <stdio.h>
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_pll.h"
#include "bsp_leds.h"
#include "hal_adc.h"
#ifdef MRFI_CC430
  #include "uart_intfc_cc430.h"
#else
  #include "uart_intfc.h"
#endif

typedef struct midimsg
{
  uint8_t cmd;
  uint8_t val1;
  uint8_t val2;
}midimsg;

typedef struct chan
{
  uint8_t note;
  uint16_t velocity;
  uint8_t on;
  uint16_t timeout;
  uint8_t delay;
}chan;

chan chans[8];

// -- ADC result table --
static __xdata uint8 adc_data[8];

void Timer4_init(void);
__interrupt void Timer_4_int(void);
#pragma vector=T4_VECTOR
__interrupt void Timer_4_int()
{
    if(TIMIF&0x10)
    {
    for (uint8 i = 0;i<8;i++)
      {
        if(chans[i].delay>0)
        chans[i].delay--;
      
        if(chans[i].timeout>0)
        chans[i].timeout--;
      
      }
      TIMIF&=~0x2;
    }
}

void main (void)
{
  /* holds length of current message */
  uint8_t len; 
  
  /* the link token */
  linkID_t LinkID = 0;
  
  /* the transmit and receive buffers */
  uint8_t rx[MAX_APP_PAYLOAD], tx[MAX_APP_PAYLOAD];

  /* holds led indicator time out counts */
  uint16_t led_tmr;
  uint16_t adcread=0;
  midimsg midi;
  
  BSP_Init( );
  
  SMPL_Init( NULL );
  
  uart_intfc_init( );

  /* turn on the radio so we are always able to receive data asynchronously */
  SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, NULL );
  
  /* turn on LED. */
  BSP_TURN_ON_LED1( );
  HalAdcInit();
 // P1SEL=0;
  //P1DIR|=0x7;
  DMA_init(adc_data);
  Timer4_init();

#ifdef  LINK_TO
  {
    uint8_t cnt = 0;
    tx_send_wait( "Linking to...\r\n", 15 );
    while (SMPL_SUCCESS != SMPL_Link(&LinkID))
      if( cnt++ == 0 )
      {
        /* blink LED until we link successfully */
        BSP_TOGGLE_LED1( );
      }
  }
#else // ifdef LINK_LISTEN
  tx_send_wait( "Listening for Link...\r\n", 23 );
  while (SMPL_SUCCESS != SMPL_LinkListen(&LinkID))
  {
    /* blink LED until we link successfully */
    BSP_TOGGLE_LED1( );
  }
#endif

  tx_send_wait( "Link Established!\r\nReady...\r\n", 29 );
  
  /* turn off the led */
  BSP_TURN_OFF_LED1( );
  
  main_loop:
    /* Check to see if we received any characters from the other radio
     * and if we have send them out the uart and reset indicator timeout.
     * Prioritize on radio received links as that buffer is the most time
     * critical with respect to the UART buffers.
     */
    if( SMPL_Receive( LinkID, tx, &len ) == SMPL_SUCCESS )
    {
      /* blocking call but should be ok if both ends have same uart buad rate */
      tx_send_wait( tx, len );
      led_tmr = INDICATOR_TIME_OUT;   /* update activity time out */
    }

  /*  adcread=HalAdcRead(5,HAL_ADC_RESOLUTION_10);
    
    
    if (adcread>10) 
    {
      adcread=0x4455;
      
    }
    
    while(1)
    {
      for(uint8_t i =0; i<8; i++)
      {
        P1=(0xF8|i);
        adcread=HalAdcRead(5,HAL_ADC_RESOLUTION_10);
        if (adcread>10) 
        {
          if(chans[i].timeout==0)
          {
            if (chans[i].velocity<adcread)
            {
              chans[i].velocity=adcread;
            }
          }
          else
          {
            
          }
         (uint8_t)((float)adcread/511*127)
          while( SMPL_Send( LinkID, (uint8_t*)&midi, sizeof(midimsg) ) != SMPL_SUCCESS );
        }
        
      }
      
    }
    
    
      while( SMPL_Send( LinkID, (uint8_t*)&midi, sizeof(midimsg) ) != SMPL_SUCCESS );
     */
    while(1){
    HalAdcDMA();
     for(uint8_t i =0; i<8; i++)
      {
          if(chans[i].timeout==0)
          {
           
            if(!chans[i].on)
            {
              if(!chans[i].delay)
              {
              chans[i].on=1;
              chans[i].timeout=4;
              }
            }
            else
            {
              midi.cmd=0x90;
              midi.val1=chans[i].note;
              midi.val2=chans[i].velocity;
              SMPL_Send( LinkID, (uint8_t*)&midi, sizeof(midimsg));
              chans[i].on=0;
              chans[i].delay=50;
              chans[i].velocity=0;
            }
          }
          if (chans[i].on)
          {
              chans[i].velocity=max(chans[i].velocity,adc_data[i]);
           
          }
          
        }
    
    }
   /*
    SMPL_Send( LinkID, (uint8_t*)&adc_data, sizeof(adc_data));
    adcread=HalAdcRead(5,HAL_ADC_RESOLUTION_8);
    while ( DMAIRQ != DMA_CHANNEL_0 );
    DMAARM = DMA_CHANNEL_0;
    */
    FHSS_ACTIVE( if( nwk_pllBackgrounder( false ) != false ) );
    {
      /* check to see if the host has sent any characters and if it has
      * then send them over the radio link and reset indicator timeout.
      */
      len = rx_receive( rx, MAX_APP_PAYLOAD );
      if( len != 0 )
      {
        while( SMPL_Send( LinkID, rx, len ) != SMPL_SUCCESS )
          ;
        led_tmr = INDICATOR_TIME_OUT;   /* update activity time out */
        
        /* By forcing a minimum delay between transmissions we guarantee
        * a window for receiving packets.  This is necessary as the radio
        * link is half duplex while the UART is full duplex.  Without this
        * delay mechanism, packets can get lost as both ends may attempt to
        * transmit at the same time which the CCA algorithm fails to handle.
        */
        MRFI_DelayMs( 5 );
      }
    }
    
    /* manage led indicator */
    if( led_tmr != 0 )
    {
      led_tmr--;
      BSP_TURN_ON_LED1( );
    }
    else
      BSP_TURN_OFF_LED1( );

    goto main_loop; /* do it again and again and again and ... */
}


void Timer4_init()
{
  
  T4CTL = 0xFE; //modulo to t3cc0, pre=128, start timer
  T4CC0 = 0xFA;
  T4CCTL0 =  0x44;  
    /*Enable Timer3 Overflow Interrupt*/
  TIMIF &= ~0x01; // Clear Flag
  IEN1 |= 0x10; // T4IE = 1 

    // Enable Global Interrupts
  EA = 1;
  
}

