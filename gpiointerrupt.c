/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>


/* Example/Board Header files */
#include "Board.h"

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)

#define THREADSTACKSIZE   (768)


#define Board_ADC5              CC1350_LAUNCHXL_433_ADC5
#define Board_ADC6              CC1350_LAUNCHXL_433_ADC6

/* ADC conversion result variables */
uint16_t adcValue0, adcValueLight;
//uint32_t adcValue0MicroVolt;
uint16_t adcValue1[ADC_SAMPLE_COUNT];
uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];


ADC_Handle   adc, adc_light;
ADC_Params   params;
int_fast16_t res, res_light;



/*pin driver handle */
static PIN_State    sensingPinState;
static PIN_Handle   sensingPinHandle;

static PIN_State    buttonPinState;
static PIN_Handle   buttonPinHandle;


static uint32_t uDisableAdcChannels = 0x70; // 마스크를 씌워 사용 0x70은 핀 번호에 의해 설정 센서들은 low일 때 사용가능이기 때문에 && 111을 하여  high로 전환
static uint32_t uEnableAdcChannels = 0x00; // 반대로 0은 low이기 때문에


/* pin table */
static PIN_Config sensingPinTable[] =
{
     IOID_4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /*Temperature */
     IOID_5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /*Smoke-light initially off */
     IOID_6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /*Smoke */
     PIN_TERMINATE                                                            /*Terminate list */
};


//button table
static PIN_Config buttonPinTable[] =
{
     Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP  | PIN_IRQ_POSEDGE,
     PIN_TERMINATE
};

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
 */
void gpioButtonFxn0(uint_least16_t index)
{
    CPUdelay(300 * 1000 * 48 / 4);       // delay 300ms

    PIN_setPortOutputValue(sensingPinHandle, uEnableAdcChannels);
    //printf("pin:  %d\n", res2);


    /* Blocking mode conversion */
    res = ADC_convert(adc, &adcValue0);
    res_light = ADC_convert(adc_light, &adcValueLight);



    if(res == ADC_STATUS_SUCCESS && res_light == ADC_STATUS_SUCCESS)
    {

       // printf("ADC0 convert succeed\n");
        printf("temperature adcvalue result  : %d\n", adcValue0);
        printf("smoke adcvalue result : %d\n", adcValueLight);


    }
    else
    {

        printf("ADC0 convert failed\n");

    }


    ADC_close(adc);


    PIN_setPortOutputValue(sensingPinHandle, uDisableAdcChannels);
}


void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    ADC_init();


    ADC_Params_init(&params);


    /* Configure button pins */
//    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    //    GPIO_setConfig(Board_GPIO_BUTTON0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);

    buttonPinHandle = PIN_open(&buttonPinState , buttonPinTable);

//    PIN_registerIntCb(buttonPinHandle, &gpioButtonFxn0);

//    PIN_setInterrupt(sensingPinHandle, PIN_IRQ_BOTHEDGES );


    adc = ADC_open(Board_ADC6, &params);
    adc_light = ADC_open(Board_ADC5, &params);


    if (adc == NULL || adc_light == NULL)
    {
        printf("error adc0\n");
        while (1);
    }


    sensingPinHandle = PIN_open(&sensingPinState , sensingPinTable);

//    /* install Button callback */
//    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

//    /* Enable interrupts */
//    GPIO_enableInt(Board_GPIO_BUTTON0);

    //return (NULL);
}
