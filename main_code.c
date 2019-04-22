///
/// @file
/// @copyright All code copyright Movidius Ltd 2014, all rights reserved.
///            For License Warranty see: common/license.txt
///
/// @brief
///

// 1: Includes
// ----------------------------------------------------------------------------

#include "stdio.h"
#include "string.h"

#include "app_config.h"
#ifdef MV0212
#include "MV0212.h"
#include "brdGpioCfgs/brdMv0182R5GpioDefaults.h"
#elif defined(MV0182)
#include <Board182Api.h>
#endif
#include "DrvCDCEL.h"
#include "DrvCpr.h"
#include "DrvGpioDefines.h"
#include "registersMyriad.h"
#include "DrvADV7513.h"
#include "CamGenericApi.h"
#include "LcdApi.h"
#include "DrvMss.h"
#include "DrvRegUtilsDefines.h"

#include "LcdCEA1080p60.h"
#include "imx208_2L_1936x1096_Raw10Converted8_60Hz.h"


// 2:  Source specific #defines and types  (typedef, enum, struct)
// ----------------------------------------------------------------------------

#define MAX_USED_BUF             3   //minimum 2 for ping-pong
#define FIRST_INCOMING_BUF_ID    1   //this is in fact 2, because CamGeneric call once the getFrame fc. before start
#define FIRST_OUTGOING_BUF_ID    0

#define CAM_WINDOW_START_COLUMN  0
#define CAM_WINDOW_START_ROW     8   //cut the embedded info of the first 8 lines

#define WINDOW_WIDTH             1920
#define WINDOW_HEIGHT            1080

#define CAM_BPP 1                //RAW10 cut down to 8 bits by the SIPP filter : a simple solution available for this black&white sensor,
                                 //which avoid the usage of a debayer filter
#define LCD_BPP 1                //YUV422p: the planar YUV422p type has 1 bpp for EVERY PLANE (indication to every plane DMAs,
                                 //not an indicator for global storage)

#define CAM_FRAME_SIZE_BYTES     (WINDOW_WIDTH * WINDOW_HEIGHT * CAM_BPP)
#define LCD_CHROMA_SIZE_BYTES    (WINDOW_WIDTH * WINDOW_HEIGHT * LCD_BPP )  //size of a single chroma plane

#define DDR_AREA                 __attribute__((section(".ddr.bss")))

#ifdef MV0212
#define NUM_I2C_DEVS 3
#endif
// 3: Global Data (Only if absolutely necessary)
// ----------------------------------------------------------------------------
static I2CM_Device * i2c0Handle;
static I2CM_Device * i2c2Handle;

// 4: Static Local Data
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
GenericCamSpec *camConfig = &imx208_2L_1936x1096_RAW10CONV8_60Hz_camCfg;
ADV7513ContfigMode_t ADV7513Config = ADV7513_1080P60;

volatile u32 newCamFrameCtr = FIRST_INCOMING_BUF_ID;

u8 DDR_AREA  camBuf[MAX_USED_BUF][CAM_FRAME_SIZE_BYTES];
u8 DDR_AREA  lcdDummyChroma[LCD_CHROMA_SIZE_BYTES];

GenericCameraHandle  camHndl;
CamUserSpec          userConfig;
frameSpec            camFrameSpec;
frameBuffer          camFrame[MAX_USED_BUF];
frameSpec            lcdFrameSpec;
frameBuffer          lcdFrame[MAX_USED_BUF];
LCDHandle            lcdHndl;

callbacksListStruct          callbacks = {0};
interruptsCallbacksListType  isrCallbacks = {0};

// 5: Static Function Prototypes
// ----------------------------------------------------------------------------
frameBuffer* AllocateNextCamFrameBuf(void);

// 6: Functions Implementation
// ----------------------------------------------------------------------------
void prepareDriverData(void)
{
   int ibuf;


   //user extra settings
   userConfig.mipiControllerNb  = CAM_B1_MIPICTRL;                     //the SIPP camera connected to 1st position of CAMB connector
   userConfig.receiverId        = SIPP_DEVICE1;                        //
#ifdef MV0212
   userConfig.sensorResetPin    = MV0212_MV0200_SENSOR_LEFT_RST_GPIO;
#elif defined(MV0182)
   userConfig.sensorResetPin    = MV0182_MV0200_SENSOR_LEFT_RST_GPIO;   //
#endif
   userConfig.stereoPairIndex   = CAM_B1_LEFT_ADDR;                    //
   userConfig.windowColumnStart = CAM_WINDOW_START_COLUMN;
   userConfig.windowRowStart    = CAM_WINDOW_START_ROW;
   userConfig.windowWidth       = WINDOW_WIDTH;
   userConfig.windowHeight      = WINDOW_HEIGHT;
   userConfig.generateSync      = NULL;

   //synchronize local buffers spec with driver configuration
   camFrameSpec.width   = WINDOW_WIDTH;
   camFrameSpec.height  = WINDOW_HEIGHT;
   camFrameSpec.bytesPP = CAM_BPP;
   camFrameSpec.stride  = WINDOW_WIDTH * CAM_BPP;
   camFrameSpec.type    = YUV420p;

   printf("Size of CamFrame : %d\n", sizeof(unsigned char*));
   //prepare output buffers
   for (ibuf = 0; ibuf < MAX_USED_BUF;ibuf++)
   {
      camFrame[ibuf].p1    = (unsigned char*)camBuf[ibuf];
      camFrame[ibuf].p2    = (unsigned char*)camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT;
      camFrame[ibuf].p3    = (unsigned char*)camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT + WINDOW_WIDTH * WINDOW_HEIGHT /4;
      camFrame[ibuf].spec  = camFrameSpec;
      printf("ADDRESS OF Cam.p1 = %x Value = %d \n", &camBuf[ibuf][0],camBuf[ibuf][0]);
      printf("ADDRESS OF Cam.p2 = %x Value = %d\n",camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT, *(camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT));
      printf("ADDRESS OF Cam.p3 = %x Value = %d\n", camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT + WINDOW_WIDTH * WINDOW_HEIGHT /4, *(camBuf[ibuf]+ WINDOW_WIDTH * WINDOW_HEIGHT + WINDOW_WIDTH * WINDOW_HEIGHT /4));
   }

  /* printf("Address of CamFrame = %x \n ", camFrame[0].p1);
   printf("Address of CamFrame = %x \n ", camFrame[0].p1);
   printf("Address of CamFrame = %x \n ", camFrame[0].p1);
*/
   isrCallbacks.getBlock     = NULL;
   isrCallbacks.getFrame     = AllocateNextCamFrameBuf;
   isrCallbacks.notification = NULL;
   callbacks.isrCbfList      = &isrCallbacks;
   callbacks.sensorCbfList   = NULL;

   //initialize the LCD data
   lcdFrameSpec.width   = WINDOW_WIDTH;
   lcdFrameSpec.height  = WINDOW_HEIGHT;
   lcdFrameSpec.stride  = WINDOW_WIDTH * LCD_BPP;
   lcdFrameSpec.bytesPP = LCD_BPP;
   lcdFrameSpec.type    = YUV420p;

   for (ibuf = 0; ibuf < MAX_USED_BUF;ibuf++)
   {
       lcdFrame[ibuf].spec = lcdFrameSpec;
       lcdFrame[ibuf].p1   = &camBuf[ibuf][0];  //use for lcd directly the camera buffers, as Y plane
       lcdFrame[ibuf].p2   = &camBuf[ibuf][WINDOW_HEIGHT * WINDOW_WIDTH];    //use dummy UV planes
       lcdFrame[ibuf].p3   = &camBuf[ibuf][WINDOW_WIDTH * WINDOW_HEIGHT + WINDOW_WIDTH * WINDOW_HEIGHT/4];//lcdDummyChroma;    //

      printf("ADDRESS OF LCD.p1 = %x \n", &camBuf[ibuf][0]);
      printf("ADDRESS OF LCD.p2 = %x \n", &camBuf[ibuf][WINDOW_HEIGHT * WINDOW_WIDTH]);
      printf("ADDRESS OF LCD.p3 = %x \n", &camBuf[ibuf][WINDOW_WIDTH * WINDOW_HEIGHT + WINDOW_WIDTH * WINDOW_HEIGHT/4]);


   }
/*
   printf("Address of LCDFrame = %x \n ", lcdFrame[0].p1);
   printf("Address of LCDFrame = %x \n ", lcdFrame[0].p2);
   printf("Address of LCDFrame = %x \n ", lcdFrame[0].p3);
*/

   return;
}


frameBuffer* AllocateNextCamFrameBuf(void)
{
   newCamFrameCtr++;
   return ( &camFrame[newCamFrameCtr % MAX_USED_BUF] );
/*
	 // preserve the index of the MRU frame as j
	   int j = newCamFrameCtr++;
	   int y0,y1,x0,x1;

	   if( newCamFrameCtr == MAX_USED_BUF )
	     newCamFrameCtr = 0;
	   // draw a 2px red bounding box between (y0,x0)-(y1,x1)
	   // note, because subsampling, we round down y0/x0 to even, and we round up y1/x1 to odd
	   u8 *const pY = camFrame[j].p1;
	   u8 *const pU = camFrame[j].p2, *const pV = camFrame[j].p3;
	   const int ytop = y0 & ~1;
	   const int xleft = x0 & ~1;
	   const int ybottom = y1 | 1;
	   const int xright = x1 | 1;
	   int y, x, n;
	   // draw the top 2 rows
	   // - Y plane: 2 rows (y0&~1 and y0|1)
	   y = ytop, x = xleft, n = xright - xleft + 1;
	   memset( pY + y * WINDOW_WIDTH + xl, RED_Y, n );
	   memset( pY +( y + 1 )* WINDOW_WIDTH + xl, RED_Y, n );
	   // - U and V planes: single row (y0>>1)
	   y >>= 1; x >>= 1; n =(( n - 1 )>> 1 )+ 1;
	   memset( pU + y *( WINDOW_WIDTH >> 1 )+ x, RED_U, n );
	   memset( pV + y *( WINDOW_WIDTH >> 1 )+ x, RED_V, n );
	   // draw the bottom 2 rows
	   // - Y plane: 2 rows (y1&~1 and y1|1)
	   y = ybottom & ~1, x = xleft, n = xright - xleft + 1;
	   memset( pY + y * WINDOW_WIDTH + x, RED_Y, n );
	   memset( pY +( y + 1 )* WINDOW_WIDTH + x, RED_Y, n );
	   // - U and V planes: single row (y1>>1)
	   y >>= 1; x >>= 1; n =(( n - 1 )>> 1 )+ 1;
	   memset( pU + y *( WINDOW_WIDTH >> 1 )+ x, RED_U, n );
	   memset( pV + y *( WINDOW_WIDTH >> 1 )+ x, RED_V, n );
	   // draw the vertical rows
	   const int xl1 = xleft, xl2 = xleft | 1;
	   const int xr1 = xright & ~1, xr2 = xright;
	   const int xl_chr = xl1 >> 1, xr_chr = xr1 >> 1;
	   for( y = ytop + 2, y_chr =( ytop >> 1 )+ 1; y < ybottom; y += 2, ++y_chr ) {
	     pY[ y * WINDOW_WIDTH + xl1 ] = RED_Y;
	     pY[ y * WINDOW_WIDTH + xl2 ] = RED_Y;
	     pY[ y * WINDOW_WIDTH + xr1 ] = RED_Y;
	     pY[ y * WINDOW_WIDTH + xr2 ] = RED_Y;
	     pY[( y + 1 )* WINDOW_WIDTH + xl1 ] = RED_Y;
	     pY[( y + 1 )* WINDOW_WIDTH + xl2 ] = RED_Y;
	     pY[( y + 1 )* WINDOW_WIDTH + xr1 ] = RED_Y;
	     pY[( y + 1 )* WINDOW_WIDTH + xr2] = RED_Y;
	     pU[ y_chr *( WINDOW_WIDTH >> 1 )+ xl_chr ] = RED_U;
	     pU[ y_chr *( WINDOW_WIDTH >> 1 )+ xr_chr ] = RED_U;
	     pV[ y_chr *( WINDOW_WIDTH >> 1 )+ xl_chr ] = RED_V;
	     pV[ y_chr *( WINDOW_WIDTH >> 1 )+ xr_chr ] = RED_V;
	   }
	   // HERE: if the L2 cache is in write-BACK mode, need to flush it
	   // return a pointer to the new frame
	   return ( &camFrame[newCamFrameCtr] );

*/

}

frameBuffer* allocateLcdFrame(int layer)
{
    (void) layer;// "use" the variables to hush the compiler warning.

   return ( &lcdFrame[(newCamFrameCtr - 1)  % MAX_USED_BUF] );
}


int main()
{
    s32 status;
    camErrorType camStatus;
    LCDLayerOffset lcdLayerStartOffset = {0, 0};
    s32 boardStatus;

    status = initClocksAndMemory();
    if(status)
        return status;

#ifdef MV0212
    int32_t rc;
    uint32_t rev;
    BoardI2CInfo info[NUM_I2C_DEVS];
    BoardConfigDesc config[] =
    {
        {
            BRDCONFIG_GPIO,
            // use this for the sake of testing as it has the same gpio config as the MV0212 R0
            (void *)brdMV0182R5GpioCfgDefault
        },
        {
            BRDCONFIG_END,
            NULL
        }
    };

    rc = BoardInit(config);
    if (rc!=BRDCONFIG_SUCCESS)
    {
    	printf("Error: board initialization failed with %ld status\n",
    	    			rc);
        return rc;
    }

    rc = BoardGetPCBRevision(&rev);
    if (rc!=BRDCONFIG_SUCCESS)
    {
    	printf("Error: board configuration read failed with %ld status\n",
    	    			rc);
        return rc;
    }
    printf("Board Mv0212 initialized, revision = %lu \n", rev);

    boardStatus = BoardInitExtPll(EXT_PLL_CFG_148_24_24MHZ);
    if (boardStatus != BRDCONFIG_SUCCESS)
    {
    	printf("Error: board initialization failed with %ld status\n",
    			boardStatus);
    	return -1;
    }
    rc = BoardGetI2CInfo(info, NUM_I2C_DEVS);
    i2c0Handle=info[0].handler;
    i2c2Handle=info[2].handler;
#elif defined(MV0182)
    boardStatus = BoardInitialise(EXT_PLL_CFG_148_24_24MHZ);
    i2c0Handle=gAppDevHndls.i2c0Handle;
    i2c2Handle=gAppDevHndls.i2c2Handle;
    if (boardStatus != B_SUCCESS)
    {
    	printf("Error: board initialization failed with %ld status\n",
    			boardStatus);
    	return -1;
    }
#endif

    printf("Configuring camera and datapath\n");
    prepareDriverData();

    //prefill the common UV buffer of the output YUV422p with constant neutral pattern(luma Y plane will be overwritten by the SIPP DMA);
    //this is acceptable for the imx208 sensor, which output grayscale image, so only luma is valuable, the chroma can be neutral.

    printf("size of my memset = %d \n",sizeof(lcdDummyChroma));
    memset(lcdDummyChroma, 0x80, sizeof(lcdDummyChroma));


    //connect the LCD to output parallel bus (by GPIO) instead of MIPI Tx
    DrvMssConnectLcdToGpio();

    camStatus = CamInit( &camHndl, camConfig, &userConfig, &callbacks, i2c0Handle );
    if (camStatus != CAM_SUCCESS)
    {
        printf("\n Camera configuration failed (%d).\n", camStatus);
        return -1;
    }

    camStatus = CamStart( &camHndl );
    if (camStatus != CAM_SUCCESS)
    {
        printf("\n Camera failed to start (%d).\n", camStatus);
        return -1;
    }

    printf("Configuring the HDMI chip ADV7513\n");
    status = initADV7513reg(i2c2Handle, ADV7513Config);

    if (status != 0)
    {
        printf("\n ADV7513 chip configuration failed with code %ld.\n", status);
        return -1;
    }

    printf("Configuring the LCD\n");
    LCDInit(&lcdHndl, &lcdSpec1080p60, NULL, LCD1);
    LCDInitLayer(&lcdHndl, VL1, &lcdFrameSpec, lcdLayerStartOffset);
    LCDSetupCallbacks(&lcdHndl, &allocateLcdFrame, NULL, NULL, NULL);
    LCDStart(&lcdHndl);

    printf("\n  Streaming ... \n");
    /*for(int ibuf =0; ibuf<MAX_USED_BUF;ibuf++)
    {
      for(int i = 0; i < 12000;i++ )
         {
          camFrame[ibuf].p1[i]=76;
       	 }


      for(int i=0;i<12000/4;i++)
      {
    	  camFrame[ibuf].p2[i]=84;
    	  camFrame[ibuf].p3[i]=255;
      }
    }*/


    while(1);

    return 0;
}
