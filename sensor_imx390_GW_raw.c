
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include "sensor_imx390_GW_raw.h"
#include "../hardware.h"
#include "imx390_GW_reg.h"
/*************************************************************************************************************
**                               Global data & Function declaration
*************************************************************************************************************/

/* TI FPD Link III 954 deser I2C address */
#define TI954_ADDR_WR  (0x60)
#define TI954_ADDR_RD  (0x61)
/* TI FPD Link III 954 ser I2C address */
#define TI953_ADDR_WR  (0x30)
#define TI953_ADDR_RD  (0x31)

/*************************************************************************************************************
**                                  Function definition
*************************************************************************************************************/

void IMX390_GW_delay(int time)
{
    while(time--);
}

static CyU3PReturnStatus_t TI954_SensorWrite(uint8_t Addr, uint8_t Data)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t Buf[2];

	preamble.buffer[1] = Addr;
	preamble.buffer[0] = TI954_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 2;
	preamble.ctrlMask = 0x0000;
	Buf[0] = Data;
	apiRetStatus = CyU3PI2cTransmitBytes (&preamble, Buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(800); /* known issue for SDK I2C */
		//CyU3PDebugPrint (4, "TI954 Write Success, Success Code = %d\r\n", apiRetStatus);
	}
	else
	{
		CyU3PDebugPrint (4, "TI954 Write Error, Error Code = %d\r\n", apiRetStatus);
	}

	return apiRetStatus;
}

static CyU3PReturnStatus_t TI954_SensorRead(uint8_t Addr, uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

	preamble.buffer[1] = Addr;
	preamble.buffer[2] = TI954_ADDR_RD; /* Slave address: Read operation */
	preamble.buffer[0] = TI954_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 3;
	preamble.ctrlMask = 0x0002;
	apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(100);
		CyU3PDebugPrint (4, "TI954 Read Success, Reg(0x%x) = 0x%x\r\n", Addr, *buf);
	}
	else
	{
		CyU3PDebugPrint (4, "TI954 Read Error, Error Code = %d\r\n", apiRetStatus);
	}

	return apiRetStatus;
}

static CyU3PReturnStatus_t TI953_SensorWrite(uint8_t Addr, uint8_t Data)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t Buf[2];

	preamble.buffer[1] = Addr;
	preamble.buffer[0] = TI953_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 2;
	preamble.ctrlMask = 0x0000;
	Buf[0] = Data;
	apiRetStatus = CyU3PI2cTransmitBytes (&preamble, Buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(100); /* known issue for SDK I2C */
		//CyU3PDebugPrint (4, "TI953 Write Success, Success Code = %d\r\n", apiRetStatus);
	}
	else
	{
		CyU3PDebugPrint (4, "TI953 Write Error, Error Code = %d\r\n", apiRetStatus);
	}

	return apiRetStatus;
}

static CyU3PReturnStatus_t TI953_SensorRead(uint8_t Addr, uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

	preamble.buffer[1] = Addr;
	preamble.buffer[2] = TI953_ADDR_RD; /* Slave address: Read operation */
	preamble.buffer[0] = TI953_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 3;
	preamble.ctrlMask = 0x0002;
	apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(100);
		CyU3PDebugPrint (4, "TI953 Read Success, Reg(0x%x) = 0x%x\r\n", Addr, *buf);
	}
	else
	{
		CyU3PDebugPrint (4, "TI953 Read Error, Error Code = %d\r\n", apiRetStatus);
	}

	return apiRetStatus;
}

CyU3PReturnStatus_t IMX390_GW_SensorWrite(uint8_t HighAddr, uint8_t LowAddr,uint8_t Data)
{

    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t Buf[2];

	preamble.buffer[1] = HighAddr;
	preamble.buffer[2] = LowAddr;
	preamble.buffer[0] = SENSOR_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 3;
	preamble.ctrlMask = 0x0000;
	Buf[0] = Data;
	apiRetStatus = CyU3PI2cTransmitBytes (&preamble, Buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(100); /* known issue for SDK I2C */
		//CyU3PDebugPrint (4, "IMX390_GW Sensor Write Success, Success Code = %d\n", apiRetStatus);
	}
	else
	{
		CyU3PDebugPrint (4, "IMX390_GW Sensor Write Error, Error Code = %d, addr = 0x%x, val = 0x%x\r\n",
				apiRetStatus, (HighAddr << 8) + LowAddr, Data);
	}

	return apiRetStatus;
}

CyU3PReturnStatus_t IMX390_GW_SensorRead(uint8_t HighAddr, uint8_t LowAddr, uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

	preamble.buffer[1] = HighAddr;
	preamble.buffer[2] = LowAddr;
	preamble.buffer[3] = SENSOR_ADDR_RD; /* Slave address: Read operation */
	preamble.buffer[0] = SENSOR_ADDR_WR; /* Slave address: Write operation */
	preamble.length = 4;
	preamble.ctrlMask = 0x0004;
	apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 1, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		IMX390_GW_delay(100);
	}
	else
	{
		CyU3PDebugPrint (4, "IMX390_GW Read Error, Error Code = %d\r\n", apiRetStatus);
	}

	return apiRetStatus;
}

static void IMX390_GW_GIO_INIT(void)
{
    CyU3PReturnStatus_t          apiRetStatus;
    CyU3PGpioSimpleConfig_t      gpioConfig;
	
    /* CTL pins are restricted and cannot be configured using I/O matrix configuration function,
     * must use GpioOverride to configure it,*/
    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_STANDBY, CyTrue)|\
                   CyU3PDeviceGpioOverride (SENSOR_CMOS_OE, CyTrue);

    if (apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO Override failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    gpioConfig.outValue    = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus           = CyU3PGpioSetSimpleConfig (SENSOR_STANDBY, &gpioConfig)|\
		                     CyU3PGpioSetSimpleConfig (SENSOR_CMOS_OE, &gpioConfig);
	
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "xx GPIO Set Config Error, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
}


/*
 * This function performs the Sensor reset.
 */
void IMX390_GW_sensor_reset(void)
{
	CyU3PReturnStatus_t apiRetStatus;
	uint8_t buf;

	/* This is used to reset TI 954 through CMOS_RST -----> TI 954 PDB */
	apiRetStatus = CyU3PGpioSetValue (SENSOR_RESET_GPIO_LI_USB30_V12, CyFalse);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\r\n",apiRetStatus);
		return ;
	}

	/* Wait for some time */
	CyU3PThreadSleep(100);

	apiRetStatus = CyU3PGpioSetValue (SENSOR_RESET_GPIO_LI_USB30_V12, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\r\n",apiRetStatus);
		return ;
	}
	/* This is used to reset TI 954 through CMOS_RST -----> TI 954 PDB */

	/* Must sleep more time for TI 954 and TI 953 is reset */
	CyU3PThreadSleep(1000);

	/* This is used to init the TI 954 register setting */
	TI954_SensorWrite(0xB3, 0x00);
	TI954_SensorWrite(0x4C, 0x01);
	TI954_SensorWrite(0x58, 0x5E);
	TI954_SensorWrite(0x5B, 0x30);  // TI 953 slave address, it is able to automatically get from TI 953
	TI954_SensorWrite(0x5C, 0x30);  // TI 953 alias address
	TI954_SensorWrite(0x5D, SENSOR_ADDR_WR);  // CAM salve address
	TI954_SensorWrite(0x65, SENSOR_ADDR_WR);  // CAM alias address

#if 0
	TI954_SensorWrite(0x4C, 0x12);
	TI954_SensorWrite(0x58, 0x5E);
	TI954_SensorWrite(0x5B, 0x30);  // TI 953 slave address, it is able to automatically get from TI 953
	TI954_SensorWrite(0x5C, 0x32);  // TI 953 alias address
	TI954_SensorWrite(0x5D, SENSOR_ADDR_WR);  // CAM salve address
	TI954_SensorWrite(0x65, SENSOR_ADDR_WR + 2);  // CAM alias address
#endif

	//TI954_SensorWrite(0x1F, 0x03); // 400Mbps csi clock per lane
	TI954_SensorWrite(0x32, 0x01);
	TI954_SensorWrite(0x33, 0x03);
	TI954_SensorWrite(0x20, 0x20);
	/* This is used to init the TI 954 register setting */

	/* GPIOs setting, and configure the GPIO2 as input HIGH */
	TI954_SensorWrite(0x0F, 0x7F);  // GPIOs as input
	TI954_SensorWrite(0x6E, 0x10);  // GPIO0->GPIO0, GPIO1->GPIO1
	TI954_SensorWrite(0x6F, 0x32);  // GPIO2->GPIO2, GPIO3->GPIO3

#if 0
	TI954_SensorRead(0x73, &buf);
	CyU3PDebugPrint (4, "0X73 buf = %x\r\n",buf);
	TI954_SensorRead(0x74, &buf);
	CyU3PDebugPrint (4, "0X74 buf = %x\r\n",buf);
	TI954_SensorRead(0x75, &buf);
	CyU3PDebugPrint (4, "0X75 buf = %x\r\n",buf);
	TI954_SensorRead(0x76, &buf);
	CyU3PDebugPrint (4, "0X76 buf = %x\r\n",buf);
#endif

	TI953_SensorWrite(0x0E, 0xF0);  // Enable GPIOs as output
	CyU3PThreadSleep(500);
	TI953_SensorWrite(0x0D, 0xF0);  // enable remote data
	CyU3PThreadSleep(100);

	  TI953_SensorWrite(0x0E, 0xF0);  // Enable GPIOs as output
		//TI953_SensorWrite(0x0D, 0xF0);  // enable remote data
	     TI953_SensorRead(0x0E, &buf);
	     CyU3PDebugPrint (4, "0X0E buf = %X\r\n",buf);

	     CyU3PThreadSleep(150);
	     		TI953_SensorWrite(0x0D, 0xB0);
	     		TI953_SensorRead(0x0D, &buf);
	     		 CyU3PDebugPrint (4, "0X0D buf = %X\r\n",buf);
	     		CyU3PThreadSleep(150);

		CyU3PThreadSleep(150);
		TI953_SensorWrite(0x0D, 0xB4);
		TI953_SensorRead(0x0D, &buf);
		 CyU3PDebugPrint (4, "0X0D buf = %X\r\n",buf);
		CyU3PThreadSleep(150);


	//TI954_SensorWrite(0xB3, 0x03);

	apiRetStatus = CyU3PGpioSetValue (CMOS_RST, CyFalse);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\r\n",apiRetStatus);
		return ;
	}

	/* Wait for some time */
	CyU3PThreadSleep(100);

	apiRetStatus = CyU3PGpioSetValue (CMOS_RST, CyTrue);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\r\n",apiRetStatus);
		return ;
	}
	/* GPIOs setting, and configure the GPIO2 as input HIGH */

	//TI954_SensorRead(0x5B, &buf);
	//TI953_SensorRead(0x00, &buf);

#if 0  /* crosslink init */
	apiRetStatus = CyU3PGpioSetValue (CMOS_D15, CyFalse);//Enable Crosslink
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\n",apiRetStatus);
		return ;
	}

	CyU3PThreadSleep(10);

	apiRetStatus = CyU3PGpioSetValue (CMOS_D15, CyTrue);//Enable Crosslink
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\n",apiRetStatus);
		return ;
	}
#endif
	CyU3PDebugPrint (4, "11111111111111111111111111111111111111111111\r\n");
	CyU3PThreadSleep(10);

	return ;
}

CyU3PReturnStatus_t
GW5200_SensorWriteNB(uint8_t SlaveAddr, uint8_t HighAddr, uint8_t LowAddr, uint8_t count, uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus=CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    int retry = 5;


    if ( count > 64 )
    {
    	CyU3PDebugPrint (4, "ERROR: Sensor Write count > 64\n");
    	return 1;
    }
  retry:
	preamble.buffer[1] = HighAddr;
	preamble.buffer[2] = LowAddr;
	preamble.buffer[0] = SlaveAddr; /* Slave address: Write operation */
	preamble.length = 3;
	preamble.ctrlMask = 0x0000;
	apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);
	if (apiRetStatus == CY_U3P_SUCCESS)
	{
		AR0220_delay(800); /* known issue for SDK I2C */
		CyU3PDebugPrint (4, "GW5200_SensorWriteNB SUCCESSFULLY  count = %d\r\n", count);

	}
	else
	{
		if(retry > 0)
		{
			retry--;
			CyU3PThreadSleep(50);
			CyU3PDebugPrint (4, "GW5200_SensorWriteNB error!\r\n");
			goto retry;
		}
		CyU3PDebugPrint (4, "GW5200_SensorWriteNB error!\r\n");
	}

	return apiRetStatus;
}

void IMX390_GW_sensor_init(int usb_type)
{
	  int j, i;
	  BYTE AddrH,AddrL,VaL;

#if 0
	  for(j = 0; j < sizeof(IMX390_GW_1920x1080) / sizeof(struct IMX390_GW_1920x1080); j++)
	  {
		  AddrH  = (IMX390_GW_1920x1080[j].addr >>8) & 0xff;
		  AddrL  = (IMX390_GW_1920x1080[j].addr) & 0xff;
		  VaL    = (IMX390_GW_1920x1080[j].val) & 0xff;

		//  IMX390_GW_SensorWrite(AddrH, AddrL, VaL);
	  }
#endif

		uint8_t wbuf[10] = {0};
		//CyU3PThreadSleep(1000);
		wbuf[0] = 0x05;
		wbuf[1] = 0x00;
		wbuf[2] = 0x00;
		wbuf[3] = 0x00;

		wbuf[4] = 0xa1;
		wbuf[5] = 0x00;
		wbuf[6] = 0x80;
		wbuf[7] = 0x00;
		wbuf[8] = 0x00;
		wbuf[9] = 0xa0;

		GW5200_SensorWriteNB(0xda, 0x33, 0x47, 10, wbuf);

	 // CyU3PThreadSleep(3000);
	  CyU3PDebugPrint(4, "IMX390_GW-CONFIG USpeed:%d\r\n", CyU3PUsbGetSpeed());
}
		
/*
   Get the current brightness setting from the IMX390_GW sensor.
 */
uint8_t IMX390_GW_SensorGetBrightness (void)
{
    return 0;
}

/*
   Update the brightness setting for the IMX390_GW sensor.
 */
void IMX390_GW_SensorSetBrightness (uint8_t brightness)
{

}

/*Contrast 16~64 for sensor,64~255 from UVC*/
/*
   Get the current contrast setting from the IMX390_GW sensor.
 */
uint8_t IMX390_GW_SensorGetContrast (void)
{
    return 0;
}

/*
   Update the contrast setting for the IMX390_GW sensor,the min value is 16
 */
void IMX390_GW_SensorSetContrast (uint8_t contrast)
{

}

/*
   Get the current HUE setting from the IMX390_GW sensor
 */
uint8_t IMX390_GW_SensorGetHUE (void)
{
    return 0;
}

/*
   Update the HUE setting for the IMX390_GW sensor
 */
void IMX390_GW_SensorSetHUE (uint8_t HUE)
{

}

/*AWB AUTO Control */
uint8_t IMX390_GW_SensorGetAWB (void )
{
	return 0;
}

void IMX390_GW_SensorSetAWB (uint8_t enable)
{

}
/*
   AWB AUTO Control
 */
uint16_t IMX390_GW_SensorGetAWB_TMP (void)
{
    return 0;
}

void IMX390_GW_SensorSetAWB_TMP (uint16_t tmp)
{

}

/*
  Saturation Control
 */
uint8_t IMX390_GW_SensorGetSaturation (void)
{
    return 0;
}

void IMX390_GW_SensorSetSaturation (uint8_t tmp)
{

}
/*
  Sharpness Control
 */
uint8_t IMX390_GW_SensorGetSharpness (void)
{
    return 0;
}

void IMX390_GW_SensorSetSharpness (uint8_t tmp)
{

}

/*
  Gamma Control
 */
uint8_t IMX390_GW_SensorGetGamma (void)
{
    return 0;
}

void IMX390_GW_SensorSetGamma (uint8_t tmp)
{

}

/*
  Gain Control
 */
uint8_t IMX390_GW_SensorGetGain (void)
{
    return 0;
}

void IMX390_GW_SensorSetGain (uint8_t tmp)
{

}

/*
  Backlight Control
 */
uint8_t IMX390_GW_SensorGetBacklight (void)
{
    return 0;
}

void IMX390_GW_SensorSetBacklight (uint8_t tmp)
{

}

/*
  AE Mode Control
 */
uint8_t IMX390_GW_SensorGetAEMode (void)
{
    return 0;
}

void IMX390_GW_SensorSetAEMode (uint8_t enable)
{

}

/*
  Exposuretime Control
 */
uint16_t IMX390_GW_SensorGetExposuretime (void)
{
    return 0;
}

void IMX390_GW_SensorSetExposuretime (uint16_t tmp)
{

}

/*
   Power Line Frequency Control,UVC protocal,1:50Hz;2:60Hz
 */
uint8_t IMX390_GW_SensorGetPowerLineFreq (void)
{
	return 0;
}

void IMX390_GW_SensorSetPowerLineFreq (uint8_t enable)
{

}

void IMX390_GW_SetWindow(uint16_t preStartX, uint16_t preStartY, uint16_t preEndX, uint16_t preEndY)
{

}

void IMX390_GW_Snapshot_MODE(uint8_t enable)
{

}

void IMX390_GW_SetTrigger(uint8_t enable)
{

}
void IMX390_GW_SETTrigger_Vsync(uint8_t enable)
{
}

