
#include <cyu3types.h>

#define DVK_SDK_PORT
#define HD720P
#define UVC

typedef unsigned char  BYTE;
typedef unsigned int 	uint;

/* imx390_GW sensor I2C address */
#define SENSOR_ADDR_WR 0xDA
#define SENSOR_ADDR_RD 0xDB

#define IMG_SENSOR_FULL_SIZE_WIDTH  4000
#define IMG_SENSOR_FULL_SIZE_HEIGHT 2000

#define IMX390_GW_AUTO_EXPOSURE_MODE    8
#define IMX390_GW_MANUAL_EXPOSURE_MODE  1

/*GPIO22 is not used on LI-USB30 V1.0 board,coz sensor reset pin has been connected to FX3 RSTn*/
#define SENSOR_STANDBY 20
#define SENSOR_CMOS_OE 45

#define DEFAULT_EXPOSURE_TIME 	(500)
#define IMGS_CHIP_ID        (0x2401)

/***************************************************************************************************************
**                                          function declaration
***************************************************************************************************************/

CyU3PReturnStatus_t IMX390_GW_SensorWrite(uint8_t HighAddr, uint8_t LowAddr,uint8_t Data);
CyU3PReturnStatus_t IMX390_GW_SensorRead(uint8_t HighAddr, uint8_t LowAddr, uint8_t *buf);

void delay(int);
void IMX390_GW_sensor_init(int usb_type);

void IMX390_GW_sensor_reset(void);
extern void IMX390_GW_Snapshot_MODE(uint8_t enable);
extern void IMX390_GW_SetTrigger(uint8_t enable);
extern void IMX390_GW_SETTrigger_Vsync(uint8_t enable);

BYTE IMX390_GW_I2C_BUS_TEST(void);

/* Function    : IMX390_GW_SensorGetBrightness
   Description : Get the current brightness setting from the IMX390_GW sensor.
   Parameters  : None
 */
extern uint8_t IMX390_GW_SensorGetBrightness (void);

/* Function    : IMX390_GW_SensorSetBrightness
   Description : Set the desired brightness setting on the IMX390_GW sensor.
   Parameters  : brightness - Desired brightness level.
 */
extern void IMX390_GW_SensorSetBrightness (uint8_t brightness);

/* Function    : IMX390_GW_SensorGetContrast
   Description : Get the current Contrast setting from the IMX390_GW sensor.
   Parameters  : None
 */
extern uint8_t IMX390_GW_SensorGetContrast (void);

/* Function    : IMX390_GW_SensorSetContrast
   Description : Set the desired Contrast setting on the IMX390_GW sensor.
   Parameters  : Contrast - Desired Contrast level.
 */
extern void IMX390_GW_SensorSetContrast (uint8_t Contrast);

/*HUE Control*/
extern uint8_t IMX390_GW_SensorGetHUE (void);
extern void IMX390_GW_SensorSetHUE (uint8_t HUE);

/*AWB AUTO Control */
extern uint8_t IMX390_GW_SensorGetAWB (void );
extern void IMX390_GW_SensorSetAWB (uint8_t enable);

/*AWB TMP Control*/
extern uint16_t IMX390_GW_SensorGetAWB_TMP (void);
extern void IMX390_GW_SensorSetAWB_TMP (uint16_t tmp);

/*Saturation Control*/
extern uint8_t IMX390_GW_SensorGetSaturation (void);
extern void IMX390_GW_SensorSetSaturation (uint8_t tmp);

/*Sharpness Control*/
extern void IMX390_GW_SensorSetSharpness (uint8_t tmp);
extern uint8_t IMX390_GW_SensorGetSharpness (void);

/*Gamma Control*/
extern void IMX390_GW_SensorSetGamma (uint8_t tmp);
extern uint8_t IMX390_GW_SensorGetGamma (void);

/*Backlight Control*/
extern uint8_t IMX390_GW_SensorGetBacklight (void);
void IMX390_GW_SensorSetBacklight (uint8_t tmp);

/*AE Mode Control*/
extern uint8_t IMX390_GW_SensorGetAEMode (void );
extern void IMX390_GW_SensorSetAEMode (uint8_t enable);

/*Exposuretime Control*/
extern uint16_t IMX390_GW_SensorGetExposuretime (void);
void IMX390_GW_SensorSetExposuretime (uint16_t tmp);

/* Power Line Frequency Control */
extern uint8_t IMX390_GW_SensorGetPowerLineFreq (void );
extern void IMX390_GW_SensorSetPowerLineFreq (uint8_t enable);

/* Gain Control */
extern uint8_t IMX390_GW_SensorGetGain (void);
extern void IMX390_GW_SensorSetGain (uint8_t tmp);

extern void IMX390_GW_SetWindow(uint16_t preStartX, uint16_t preStartY, uint16_t preEndX, uint16_t preEndY);

