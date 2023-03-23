//Patrik Koivisto & Pyry Myllymäki

//Includes
#include <stdio.h>
#include <time.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"


/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];


//STATES
enum state { WAITING=1, DATA_READY, FEEDING, MOVING, PET, SLEEPING };
enum state programState = WAITING;

double ambientLight = -1000.0;

// GLOBALS FOR PINS
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle button1Handle;
static PIN_State button1State;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;

PIN_Config buttonConfig[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config button1Config[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {
    programState = FEEDING;
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
}

void button1Fxn(PIN_Handle handle, PIN_Id pinId) {
    programState = PET;
}

/* UartTask Functions*/
Void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;
    UART_Params_init(&uartParams);
       uartParams.writeDataMode = UART_DATA_TEXT;
       uartParams.readDataMode = UART_DATA_TEXT;
       uartParams.readEcho = UART_ECHO_OFF;
       uartParams.readMode=UART_MODE_BLOCKING;
       uartParams.baudRate = 9600; // nopeus 9600baud
       uartParams.dataLength = UART_LEN_8; // 8
       uartParams.parityType = UART_PAR_NONE; // n
       uartParams.stopBits = UART_STOP_ONE; // 1

        char feed[80];
        char moved[80];
        char pet[80];
        char sleep[80];
        while (1) {
            if(programState == FEEDING){
                uart = UART_open(Board_UART0, &uartParams);
                     if (uart == NULL) {
                     System_abort("Error opening the UART");
                     }
            sprintf(feed,"id:28, EAT:2\0");
            UART_write(uart, feed, strlen(feed));
           UART_close(uart);
            programState = WAITING;

        }
            if(programState == PET){
                uart = UART_open(Board_UART0, &uartParams);
                    if (uart == NULL) {
                        System_abort("Error opening the UART");
                     }
                sprintf(pet,"id:28, PET:2\0");
                UART_write(uart, pet, strlen(pet));
                UART_close(uart);
                programState = WAITING;
            }
            if(programState == MOVING){
              uart = UART_open(Board_UART0, &uartParams);
              if (uart == NULL) {
              System_abort("Error opening the UART");
              }
              sprintf(moved,"id:28, EXERCISE:2\0");
              UART_write(uart, moved, strlen(moved));
              UART_close(uart);
              programState = WAITING;
            }
            if(programState == SLEEPING){
                uart = UART_open(Board_UART0, &uartParams);
                if (uart == NULL) {
                System_abort("Error opening the UART");
                }
                sprintf(sleep,"id:28, ACTIVATE:1;1;1\0");
                UART_write(uart, sleep, strlen(sleep));
                UART_close(uart);
                programState = WAITING;

            }
            Task_sleep(1000000 / Clock_tickPeriod);
        }
    }

/* SensorTask Functions*/
Void sensorTaskFxn(UArg arg0, UArg arg1){
    float ax, ay, az, gx, gy, gz;
    char luxi [80];
    char liike [80];


     I2C_Handle      i2c;
     I2C_Params      i2cParams;
     I2C_Params_init(&i2cParams);
     i2cParams.bitRate = I2C_400kHz;


     I2C_Handle i2cMPU;
     I2C_Params i2cMPUParams;
     I2C_Params_init(&i2cMPUParams);
     i2cMPUParams.bitRate = I2C_400kHz;
     i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;


     // Open Connection
     i2c = I2C_open(Board_I2C_TMP, &i2cParams);
     if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
       }
       Task_sleep(100000 / Clock_tickPeriod);
       opt3001_setup(&i2c);
       System_printf("Setup ok\n");
       System_flush();
       I2C_close(i2c);

       System_printf("pins ok\n");
       System_flush();
        // MPU power on
       PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
       System_printf("MPU9250: Power ON\n");
       System_flush();
       Task_sleep(100000 / Clock_tickPeriod);
       System_printf("sleep ok\n");
       System_flush();

         i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
         if (i2cMPU == NULL) {
             System_abort("Error Initializing I2CMPU\n");
         }

         System_printf("MPU9250: Setup and calibration...\n");
         System_flush();

         mpu9250_setup(&i2cMPU);

         System_printf("MPU9250: Setup and calibration OK\n");
         System_flush();
         I2C_close(i2cMPU);


       while(1){
           if(programState == WAITING){
           i2c = I2C_open(Board_I2C, &i2cParams);
           double Light = opt3001_get_data(&i2c);
           sprintf(luxi,"%lf, sensortask",opt3001_get_data(&i2c));
           System_printf("%s\n", luxi);
           System_flush();
           I2C_close(i2c);

           float liikekok = 0;
           i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
           mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
           liikekok = fabs(ax) + fabs(ay) + fabs(az);
           sprintf(liike,"%lf, x akseli", liikekok);
           System_printf("%s\n", liike);
           I2C_close(i2cMPU);

           if(Light < 0.1){
               programState = SLEEPING;
           }
           else if(liikekok > 3){
               programState = MOVING;
           }
           Task_sleep(1000000 / Clock_tickPeriod);
           }
       }
}

Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();
    Board_initUART();
    Init6LoWPAN();
    Board_initI2C();

    ledHandle = PIN_open( &ledState, ledConfig );
        if(!ledHandle) {
           System_abort("Error initializing LED pin\n");
        }

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (hMpuPin == NULL) {
            System_abort("Error initializing failed!");
        }


    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pin\n");
    }

    button1Handle = PIN_open(&button1State, button1Config);
       if(!button1Handle) {
          System_abort("Error initializing button pin\n");
       }

    if (PIN_registerIntCb(button1Handle, &button1Fxn) != 0) {
       System_abort("Error registering button callback function");
    }
        if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
           System_abort("Error registering button callback function");
        }
    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=1; //2
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
