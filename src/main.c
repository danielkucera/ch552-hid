// CH552 Dragon (Ref. HW A031219)
//
// USB device HID demo.
// If TIN3 is touched a text is "typed" on the Host.
// The NUMLOCK led status on the host keyboard is echoed on LED2, 
// so pressing the NUMLOCK key will turn on and off LED2.
// If TIN2 is touched the bootloader will be activated
// and the CH552 will be ready for a new flash programming operation.
//
// CREDITS
// Adapted to the CH552 Dragon from the demo by Aaron Christophel
// http://atcnetz.blogspot.com/2019/02/ch552-020-mikrocontroller-mit-usb.html

typedef unsigned char                 *PUINT8;
typedef unsigned char volatile        UINT8V;

#include "ch554.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <ch554_usb.h>
#include <bootloader.h>

__xdata uint8_t readFlag = 0;	
__xdata char sPath[] = " This TEXT is typed by the CH552 Dragon ";
					
__xdata char *pStr = sPath;
uint32_t millis, last,last1;

#define LED_PIN1 1				// LED1
SBIT(LED1, 0x90, LED_PIN1);
#define LED_PIN2 7				// LED2
SBIT(LED2, 0x90, LED_PIN2);

SBIT(Ep2InKey, 0xB0, 2);

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];	  
__xdata __at (0x0050) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];   
__xdata __at (0x000a) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];	 

uint8_t   SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig;
PUINT8  pDescr;                                                             
USB_SETUP_REQ   SetupReqBuf;    


uint8_t a,b,numlock,capslock;

void jump_to_bootloader()
{
	USB_INT_EN = 0;
	USB_CTRL = 0x06;
	EA = 0;
	mDelaymS(100);
	bootloader();
	while(1);
}

void	mTimer0Interrupt( void ) __interrupt (INT_NO_TMR0)
{                                              
	TH0 = (65536 - 2000)/256;  // Reload
	TL0 = (65536 - 2000)%256;  // Reload    
	millis++;
}

#define		BIT0		(0X01)
#define		BIT1		(0X02)
#define		BIT2		(0X04)
#define		BIT3		(0X08)
#define		BIT4		(0X10)
#define		BIT5		(0X20)
#define		BIT6		(0X40)
#define		BIT7		(0X80)

/* Macro define */
#define		CHX				(0X00)				
#define		CH2				(BIT2)
#define		CH3				(BIT3)
#define		CH_FREE			(0x07)				

#define		TH_VALUE		(100)
#define		TOUCH_NUM		(0x04)
#define		SAMPLE_TIMES	(0x05)

__xdata uint8_t 	TK_Code[TOUCH_NUM] = {							
	0x03, 0x04, 										
};									// Select TIN2 and TIN3 touch inputs

__xdata uint16_t 			Key_FreeBuf[TOUCH_NUM];
__xdata UINT8V			Touch_IN;		

uint8_t TK_SelectChannel( uint8_t ch )
{
	if ( ch <= TOUCH_NUM )
	{
		TKEY_CTRL = ( TKEY_CTRL & 0XF8) | TK_Code[ch];
		return 1;
	}

	return	0;
}

uint8_t TK_Init( uint8_t channel)
{

__xdata	uint8_t 	i,j;
__xdata	uint16_t 	sum;
__xdata	uint16_t 	OverTime;
	
	P1_DIR_PU &= ~channel;
	P1_MOD_OC &= ~channel;
	TKEY_CTRL |= bTKC_2MS ;

	for ( i = 0; i < TOUCH_NUM; i++ )
	{
		sum = 0;
		j = SAMPLE_TIMES;
		TK_SelectChannel( i );
		while( j-- )
		{
			OverTime = 0;
			while( ( TKEY_CTRL & bTKC_IF ) == 0 )
			{
				if( ++OverTime == 0 )
				{
					return 0;
				}
			}
			sum += TKEY_DAT;												
		}
		Key_FreeBuf[i] = sum / SAMPLE_TIMES;
	}
	IE_TKEY = 1;    
	return 1;
}

void	TK_int_ISR( void ) __interrupt (INT_NO_TKEY)
{
__xdata	static uint8_t ch = 0;
__xdata	uint16_t KeyData;
	KeyData = TKEY_DAT;
	
	if( KeyData < ( Key_FreeBuf[ch] - TH_VALUE ) )
	{
		Touch_IN |=  1 << ( TK_Code[ch] - 1 );
	}
	if( ++ch >= TOUCH_NUM )
	{
		ch = 0;
	}	
	TK_SelectChannel( ch );
}

                                           

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0

#define 	THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define		BUFFER_SIZE				64
#define 	DUAL_BUFFER_SIZE		128
#define 	UsbSetupBuf     		((PUSB_SETUP_REQ)Ep0Buffer)

// https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2

#define		L_WIN 					0X08
#define 	L_ALT 					0X04
#define		L_SHIFT					0X02
#define 	L_CTL					0X01
#define 	R_WIN 					0X80
#define 	R_ALT 					0X40
#define 	R_SHIFT					0X20
#define 	R_CTL					0X10
#define 	SPACE					0X2C
#define		ENTER					0X28
#define     CAPSLOCK                0x39

//#define MOUSE 0hallowiegehts4vcxdrlpt8hallowiegehts4hallowiegehts5


__code uint8_t DevDesc[18] = {0x12,0x01,0x10,0x01,0x00,0x00,0x00,0x08,
                      0x3d,0x41,0x07,0x21,0x00,0x00,0x00,0x00,
                      0x00,0x01
                     };
__code uint8_t CfgDesc[59] =
{
    0x09,0x02,0x3b,0x00,0x02,0x01,0x00,0xA0,0x32,             //ÅäÖÃÃèÊö·û
    0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00,             //œÓ¿ÚÃèÊö·û,ŒüÅÌ
    0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00,             //HIDÀàÃèÊö·û
    0x07,0x05,0x81,0x03,0x08,0x00,0x0a,                       //¶ËµãÃèÊö·û
    0x09,0x04,0x01,0x00,0x01,0x03,0x01,0x02,0x00,             //œÓ¿ÚÃèÊö·û,Êó±ê
    0x09,0x21,0x10,0x01,0x00,0x01,0x22,0x34,0x00,             //HIDÀàÃèÊö·û
    0x07,0x05,0x82,0x03,0x04,0x00,0x0a                        //¶ËµãÃèÊö·û
};
__code uint8_t KeyRepDesc[62] =
{
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};
__code uint8_t MouseRepDesc[52] =
{
    0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,
    0xA1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x03,
    0x81,0x02,0x75,0x05,0x95,0x01,0x81,0x01,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
    0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x03,
    0x81,0x06,0xC0,0xC0
};

uint8_t HIDMouse[4] = {0x0,0x0,0x0,0x0};

// https://wiki.osdev.org/USB_Human_Interface_Devices#Report_format
uint8_t HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

void CH554SoftReset( )
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG	|=bSW_RESET;
}

void CH554USBDevWakeup( )
{
  UDEV_CTRL |= bUD_LOW_SPEED;
  mDelaymS(2);
  UDEV_CTRL &= ~bUD_LOW_SPEED;	
}

void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00;                                                           // ÏÈÉè¶šUSBÉè±žÄ£Êœ
    UEP2_DMA = (uint16_t)Ep2Buffer;                                                      //¶Ëµã2ÊýŸÝŽ«ÊäµØÖ·
    UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //¶Ëµã2·¢ËÍÊ¹ÄÜ 64×ÖœÚ»º³åÇø
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //¶Ëµã2×Ô¶¯·­×ªÍ¬²œ±êÖŸÎ»£¬INÊÂÎñ·µ»ØNAK
    UEP0_DMA = (uint16_t)Ep0Buffer;                                                      //¶Ëµã0ÊýŸÝŽ«ÊäµØÖ·
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //¶Ëµã0µ¥64×ÖœÚÊÕ·¢»º³åÇø
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUTÊÂÎñ·µ»ØACK£¬INÊÂÎñ·µ»ØNAK
    UEP1_DMA = (uint16_t)Ep1Buffer;                                                      //¶Ëµã1ÊýŸÝŽ«ÊäµØÖ·
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //¶Ëµã1·¢ËÍÊ¹ÄÜ 64×ÖœÚ»º³åÇø
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //¶Ëµã1×Ô¶¯·­×ªÍ¬²œ±êÖŸÎ»£¬INÊÂÎñ·µ»ØNAK	

	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS;                                                    // œûÖ¹DP/DMÏÂÀ­µç×è
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // Æô¶¯USBÉè±žŒ°DMA£¬ÔÚÖÐ¶ÏÆÚŒäÖÐ¶Ï±êÖŸÎŽÇå³ýÇ°×Ô¶¯·µ»ØNAK
	  UDEV_CTRL |= bUD_PORT_EN;                                                  // ÔÊÐíUSB¶Ë¿Ú
	  USB_INT_FG = 0xFF;                                                         // ÇåÖÐ¶Ï±êÖŸ
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}

void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey));                             
    UEP1_T_LEN = sizeof(HIDKey);                                             
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;               
}

void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDMouse, sizeof(HIDMouse));                             
    UEP2_T_LEN = sizeof(HIDMouse);                                           
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;              
}

void DeviceInterrupt(void) __interrupt (INT_NO_USB)	
{
    uint8_t len = 0;
    if(UIF_TRANSFER)                                                            //USBŽ«ÊäÍê³É±êÖŸ
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# ÖÐ¶Ï¶ËµãÉÏŽ«
            UEP2_T_LEN = 0;                                                     //Ô€Ê¹ÓÃ·¢ËÍ³€¶ÈÒ»¶šÒªÇå¿Õ
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //Èç¹û²»ÉèÖÃ×Ô¶¯·­×ªÔòÐèÒªÊÖ¶¯·­×ª
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ä¬ÈÏÓŠŽðNAK
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# ÖÐ¶Ï¶ËµãÉÏŽ«
            UEP1_T_LEN = 0;                                                     //Ô€Ê¹ÓÃ·¢ËÍ³€¶ÈÒ»¶šÒªÇå¿Õ
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //Èç¹û²»ÉèÖÃ×Ô¶¯·­×ªÔòÐèÒªÊÖ¶¯·­×ª
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ä¬ÈÏÓŠŽðNAK
            FLAG = 1;                                                           /*Ž«ÊäÍê³É±êÖŸ*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUPÊÂÎñ
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // ÏÞÖÆ×Ü³€¶È
                }
                len = 0;                                                        // Ä¬ÈÏÎª³É¹Š²¢ÇÒÉÏŽ«0³€¶È
                SetupReq = UsbSetupBuf->bRequest;								
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HIDÀàÃüÁî */
                {
									switch( SetupReq ) 
									{
										case 0x01://GetReport
												 break;
										case 0x02://GetIdle
												 break;	
										case 0x03://GetProtocol
												 break;				
										case 0x09://SetReport										
												 break;
										case 0x0A://SetIdle
												 break;	
										case 0x0B://SetProtocol
												 break;
										default:
												 len = 0xFF;  								 					            /*ÃüÁî²»Ö§³Ö*/					
												 break;
								  }	
                }
                else
                {//±ê×ŒÇëÇó
                    switch(SetupReq)                                        //ÇëÇóÂë
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //Éè±žÃèÊö·û
                            pDescr = DevDesc;                               //°ÑÉè±žÃèÊö·ûËÍµœÒª·¢ËÍµÄ»º³åÇø
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //ÅäÖÃÃèÊö·û
                            pDescr = CfgDesc;                               //°ÑÉè±žÃèÊö·ûËÍµœÒª·¢ËÍµÄ»º³åÇø
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          //±š±íÃèÊö·û
                            if(UsbSetupBuf->wIndexL == 0)                   //œÓ¿Ú0±š±íÃèÊö·û
                            {
                                pDescr = KeyRepDesc;                        //ÊýŸÝ×Œ±žÉÏŽ«
                                len = sizeof(KeyRepDesc);
                            }
                            else if(UsbSetupBuf->wIndexL == 1)              //œÓ¿Ú1±š±íÃèÊö·û
                            {
                                pDescr = MouseRepDesc;                      //ÊýŸÝ×Œ±žÉÏŽ«
                                len = sizeof(MouseRepDesc);
                                Ready = 1;                                  //Èç¹ûÓÐžü¶àœÓ¿Ú£¬žÃ±ê×ŒÎ»ÓŠžÃÔÚ×îºóÒ»žöœÓ¿ÚÅäÖÃÍê³ÉºóÓÐÐ§
                            }
                            else
                            {
                                len = 0xff;                                 //±Ÿ³ÌÐòÖ»ÓÐ2žöœÓ¿Ú£¬ÕâŸä»°Õý³£²»¿ÉÄÜÖŽÐÐ
                            }
                            break;
                        default:
                            len = 0xff;                                     //²»Ö§³ÖµÄÃüÁî»òÕß³öŽí
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //ÏÞÖÆ×Ü³€¶È
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //±ŸŽÎŽ«Êä³€¶È
                        memcpy(Ep0Buffer,pDescr,len);                        //ŒÓÔØÉÏŽ«ÊýŸÝ
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //ÔÝŽæUSBÉè±žµØÖ·
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// ¶Ëµã
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // ²»Ö§³ÖµÄ¶Ëµã
                                break;
                            }
                        }
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )// Éè±ž
                        {
													break;
                        }													
                        else
                        {
                            len = 0xFF;                                                // ²»ÊÇ¶Ëµã²»Ö§³Ö
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* ÉèÖÃÉè±ž */
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* ÉèÖÃ»œÐÑÊ¹ÄÜ±êÖŸ */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* ²Ù×÷Ê§°Ü */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* ²Ù×÷Ê§°Ü */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* ÉèÖÃ¶Ëµã */
                        {
                            if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ÉèÖÃ¶Ëµã2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* ÉèÖÃ¶Ëµã2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ÉèÖÃ¶Ëµã1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //²Ù×÷Ê§°Ü
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //²Ù×÷Ê§°Ü
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //²Ù×÷Ê§°Ü
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                           //²Ù×÷Ê§°Ü
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //°ü³€¶ÈŽíÎó
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len)                                                //ÉÏŽ«ÊýŸÝ»òÕß×ŽÌ¬œ×¶Î·µ»Ø0³€¶È°ü
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ä¬ÈÏÊýŸÝ°üÊÇDATA1£¬·µ»ØÓŠŽðACK
            }
            else
            {
                UEP0_T_LEN = 0;  //ËäÈ»ÉÐÎŽµœ×ŽÌ¬œ×¶Î£¬µ«ÊÇÌáÇ°Ô€ÖÃÉÏŽ«0³€¶ÈÊýŸÝ°üÒÔ·ÀÖ÷»úÌáÇ°œøÈë×ŽÌ¬œ×¶Î
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ä¬ÈÏÊýŸÝ°üÊÇDATA1,·µ»ØÓŠŽðACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //±ŸŽÎŽ«Êä³€¶È
                memcpy( Ep0Buffer, pDescr, len );                            //ŒÓÔØÉÏŽ«ÊýŸÝ
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //Í¬²œ±êÖŸÎ»·­×ª
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //×ŽÌ¬œ×¶ÎÍê³ÉÖÐ¶Ï»òÕßÊÇÇ¿ÖÆÉÏŽ«0³€¶ÈÊýŸÝ°üœáÊø¿ØÖÆŽ«Êä
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if(SetupReq == 0x09)
            {
                // https://wiki.osdev.org/USB_Human_Interface_Devices#LED_lamps
                numlock = Ep0Buffer[0] & 1;

                capslock = (Ep0Buffer[0] & 2) > 0;		
            }
            UEP0_CTRL ^= bUEP_R_TOG;                                     //Í¬²œ±êÖŸÎ»·­×ª						
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //ÐŽ0Çå¿ÕÖÐ¶Ï
    }
    if(UIF_BUS_RST)                                                       //Éè±žÄ£ÊœUSB×ÜÏßžŽÎ»ÖÐ¶Ï
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //?
    }
    if (UIF_SUSPEND)                                                     //USB×ÜÏß¹ÒÆð/»œÐÑÍê³É
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //?
        {
        }
    }
    else {                                                               //ÒâÍâµÄÖÐ¶Ï,²»¿ÉÄÜ·¢ÉúµÄÇé¿ö
        USB_INT_FG = 0xFF;        
    }
}

static void CommitKey(){
    mDelaymS( 10 );																			
	while(FLAG == 0);                   								
	Enp1IntIn();						
	while(FLAG == 0);   																					
	mDelaymS( 10 );
	HIDKey[0] = 0X00;     						
	HIDKey[2] = 0X00;                                              								
	while(FLAG == 0);                                           						
	Enp1IntIn();			
	while(FLAG == 0); 
}

static void SendKey ( char *p )
{

	char c = *p;
	char d = 0;
		
	if( (c >= 'a') && (c <= 'z' )){
		c = c - 'a' + 'A';
		d=1;
	}
	if(d == 0)HIDKey[0] = L_SHIFT;
	if( (c >= 'A') && (c <= 'Z' )){
		HIDKey[2] = c - 'A' + 4;
	}
	else
		if( c >= '1' && c <= '9' ){
				HIDKey[0] = 0x000;
		HIDKey[2] = c - '1' + 0X1E;}
		else
		{
		switch ( c ){
			case '`' :
				HIDKey[0] = 0X08;
				HIDKey[2] = 0X15;
				break;
			case '\\':
				HIDKey[2] = 0x31;
				break;
			case ' ':
				HIDKey[2] = SPACE;
				break;
			case '\r':
				HIDKey[2] = ENTER;
				break;
			case ':':
				HIDKey[0] = 0x02;
				HIDKey[2] = 0x33;
				break;
			case '+':
				HIDKey[0] = 0x000;
				HIDKey[2] = 0x57;
				break;
			case '?':
				HIDKey[0] = L_SHIFT;
				HIDKey[2] = 0x38;
				break;
			case '_':
				HIDKey[0] = 0X02;
				HIDKey[2] = 0X2D;
				break;
			case '/':
				HIDKey[0] = L_CTL + L_ALT;
				HIDKey[2] = 0X16;
				break;
			case '0':
				HIDKey[2] = 0X27;
				break;
			case '.':
				HIDKey[2] = 0X37;
				break;
			case '~':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X05;
				break;
			case '!':
				HIDKey[0] = L_ALT;
				HIDKey[2] = 0X08;
				break;
			default:
				break;
		}
	}
    CommitKey();
}


void HIDValueHandle(){
    if( readFlag == 1 ){ 	 	
        SendKey(pStr);																		
        pStr++;	
        if(*pStr == '\0'){
            readFlag = 0;
            b=0;	
        }		
	} else {
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;   
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;  
        if(b == 1){     
            b=0;
            pStr = sPath;
            readFlag=1;
        }			
    }	
	
}


main()
{
    CfgFsys( );                                         
    mDelaymS(5);                                                         
    mInitSTDIO( );      	
    USBDeviceInit();   
	TK_Init( BIT4+BIT5);		// Init TIN2 (P1.4) and TIN3 (P1.5)
	TK_SelectChannel(0);	
	
    P1_MOD_OC = P1_MOD_OC |(1<<LED_PIN1);
    P1_DIR_PU = P1_DIR_PU |	(1<<LED_PIN1);
    P3_MOD_OC = P3_MOD_OC |(1<<LED_PIN2);
    P3_DIR_PU = P3_DIR_PU |	(1<<LED_PIN2);

	
	TMOD = 0x11;
	TH0 = (65536 - 2000)/256;  	// for start value
	TL0 = (65536 - 2000)%256;  	// for start value 
	TR0 = 1;    				// Start timer 0 (TR1 for timer 1)
	ET0 = 1;    				// Enable Timer 0 interrupt
	EA  = 1;    				// Activate global interrupt                                                   
    UEP1_T_LEN = 0;                                                     
    UEP2_T_LEN = 0;                                                      
    FLAG = 0;
    Ready = 0;
	b=0;
    while(1)
    {
    LED2 = capslock;
    
	if (millis-last>25000){
		 LED1 = !LED1;
		 last=millis;         

		 if( Touch_IN != 0 )
			{
			//if( Touch_IN & CH2 )jump_to_bootloader();
			//if( Touch_IN & CH3 )b=1;
			Touch_IN = 0;
			}
	
	
        if(Ready)
        {
            //SendKey("a");
            //HIDValueHandle();

            HIDKey[0] = 0;
            HIDKey[2] = CAPSLOCK;
            CommitKey();

            HIDKey[0] = 0;
            HIDKey[2] = CAPSLOCK;
            CommitKey();
        }           
	}                                   
    }              
       
}