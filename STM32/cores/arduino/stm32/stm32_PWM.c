/*
  Copyright (c) 2017 Daniel Fekete

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "stm32_gpio.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint32_t pin_mask;

    uint32_t dutyCycle;
    int32_t counterCycles;
} PWM_PIN_CONFIG;

#define PWM_PIN_COUNT sizeof(variant_pin_list) / sizeof(variant_pin_list[0])

static PWM_PIN_CONFIG pwmConfigTable[3][PWM_PIN_COUNT];

PWM_PIN_CONFIG *pCurrentPwmPin;

PWM_PIN_CONFIG *pActiveTable = &pwmConfigTable[0];
PWM_PIN_CONFIG *pInactiveTable = &pwmConfigTable[1];
PWM_PIN_CONFIG *pWorkTable = &pwmConfigTable[2];

uint8_t pwmUpdateTable = 0;

TIM_HandleTypeDef *handle;

static uint8_t analogWriteResolutionBits = 8;

const uint32_t TIMER_MAX_CYCLES = 84000;
const uint32_t PWM_FREQUENCY_HZ = 1000;
const uint32_t TIMER_PERIOD = 84000;

extern void pinMode(uint8_t, uint8_t);

#define min(a, b) ((a) < (b) ? (a) : (b))

stm32_pwm_disable_callback_func stm32_pwm_disable_callback = NULL;

void (*pwmCallbackFunc)();

void pwmCallback();

void stm32_pwm_disable(GPIO_TypeDef *port, uint32_t pin);

void analogWriteResolution(int bits)
{
    analogWriteResolutionBits = bits;
}

static void initTimer()
{
    static TIM_HandleTypeDef staticHandle;

    if (handle == NULL)
    {
        handle = &staticHandle;
        pwmCallbackFunc = &pwmCallback;

        stm32_pwm_disable_callback = &stm32_pwm_disable;

#ifdef TIM2 //99% of chips have TIM2
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);

        handle->Instance = TIM2;
#else
        __HAL_RCC_TIM3_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);

        handle->Instance = TIM3;
#endif

        handle->Init.Prescaler = 0;
        handle->Init.CounterMode = TIM_COUNTERMODE_UP;
        handle->Init.Period = TIMER_PERIOD;
        handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        HAL_TIM_Base_Init(handle);

        TIM_OC_InitTypeDef sConfigOC;
        sConfigOC.OCMode = TIM_OCMODE_TIMING;
        sConfigOC.Pulse = TIMER_PERIOD + 1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        HAL_TIM_OC_ConfigChannel(handle, &sConfigOC, TIM_CHANNEL_1);

        HAL_TIM_Base_Start_IT(handle);
        HAL_TIM_OC_Start_IT(handle, TIM_CHANNEL_1);
    }
}

int pwmCompareFunction(const void *p1, const void *p2)
{
    PWM_PIN_CONFIG *l = (PWM_PIN_CONFIG *)p1;
    PWM_PIN_CONFIG *r = (PWM_PIN_CONFIG *)p2;

    if (l->port == NULL && r->port == NULL)
        return 0;
    else if (l->port == NULL && r->port != NULL)
        return 1;
    else if (l->port != NULL && r->port == NULL)
        return -1;

    return l->dutyCycle - r->dutyCycle;
}

void pwmApplyWorkTable(void)
{
    qsort(pWorkTable, PWM_PIN_COUNT, sizeof(PWM_PIN_CONFIG), pwmCompareFunction);

    memcpy(pInactiveTable, pWorkTable, sizeof(PWM_PIN_CONFIG) * PWM_PIN_COUNT);

    pwmUpdateTable = 1;
}

void pwmWrite(uint8_t pin, int dutyCycle, int frequency, int durationMillis)
{
    initTimer();

    for (size_t i = 0; i < PWM_PIN_COUNT; i++)
    {
        if (pWorkTable[i].port == NULL || (pWorkTable[i].port == variant_pin_list[pin].port && pWorkTable[i].pin_mask == variant_pin_list[pin].pin_mask))
        {
            if (pWorkTable[i].port == NULL)
            {
                pinMode(pin, OUTPUT);
            }

            if (dutyCycle == 0 || dutyCycle >= 65280)
            {
                pWorkTable[i].port = NULL;
                pWorkTable[i].pin_mask = 0;
                pWorkTable[i].dutyCycle = 0;
                pWorkTable[i].counterCycles = 0;

                digitalWrite(pin, dutyCycle == 0 ? 0 : 1);
            }
            else
            {
#ifdef STM32F0 // TODO better condition for when there is no pclk2
                uint32_t timerFreq = HAL_RCC_GetPCLK1Freq();
#else
                uint32_t timerFreq = HAL_RCC_GetPCLK2Freq();
#endif
                uint32_t waveLengthCycles = timerFreq / frequency;

                pWorkTable[i].port = variant_pin_list[pin].port;
                pWorkTable[i].pin_mask = variant_pin_list[pin].pin_mask;
                pWorkTable[i].dutyCycle = (uint64_t)waveLengthCycles * dutyCycle >> 16;

                if (durationMillis > 0)
                {
                    pWorkTable[i].counterCycles = timerFreq / 1000 * durationMillis;
                }
            }

            pwmApplyWorkTable();
            break;
        }
    }
}

extern void tone(uint8_t pin, unsigned int frequency, unsigned long durationMillis)
{
    pwmWrite(pin, 1 << 15, frequency, durationMillis);
}

void analogWrite(uint8_t pin, int value)
{
    pwmWrite(pin, ((uint32_t)value << 16) >> analogWriteResolutionBits, PWM_FREQUENCY_HZ, 0);
}

void stm32_pwm_disable(GPIO_TypeDef *port, uint32_t pin_mask)
{
    for (size_t i = 0; i < PWM_PIN_COUNT && pWorkTable[i].port != NULL; i++)
    {
        if (pWorkTable[i].port == port && pWorkTable[i].pin_mask == pin_mask)
        {
            pWorkTable[i].port = NULL;
            pWorkTable[i].pin_mask = 0;
            pWorkTable[i].dutyCycle = 0;
            pWorkTable[i].counterCycles = 0;

            pwmApplyWorkTable();

            break;
        }
    }
}

void pwmCallback()
{
    if (__HAL_TIM_GET_FLAG(handle, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(handle, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(handle, TIM_IT_UPDATE);

            if (pwmUpdateTable)
            {
                pCurrentPwmPin = pInactiveTable;
                pInactiveTable = pActiveTable;
                pActiveTable = pCurrentPwmPin;
                pwmUpdateTable = 0;
            }
            else
                pCurrentPwmPin = pActiveTable;

            // Set all configured pins to HIGH
            for (PWM_PIN_CONFIG *pinPtr = pCurrentPwmPin; pinPtr->port != NULL; pinPtr++)
                pinPtr->port->BSRR = pinPtr->pin_mask;

            if (pCurrentPwmPin->port != NULL)
                handle->Instance->CCR1 = pCurrentPwmPin->dutyCycle; // Capture compare interrupt will be called at pCurrentPwmPin->dutyCycle cycles
            else
                handle->Instance->CCR1 = handle->Instance->ARR + 1; // Capture compare interrupt will not be called
        }
    }
    else if (__HAL_TIM_GET_FLAG(handle, TIM_FLAG_CC1) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(handle, TIM_IT_CC1) != RESET)
        {
            __HAL_TIM_CLEAR_IT(handle, TIM_IT_CC1);

            while (pCurrentPwmPin->port != NULL && pCurrentPwmPin->dutyCycle <= handle->Instance->CNT)
            {
                // Set pins that have count <= Timer->CNT as LOW
                pCurrentPwmPin->port->BSRR = pCurrentPwmPin->pin_mask << 16;
                pCurrentPwmPin++;
            }

            if (pCurrentPwmPin->port != NULL)
                handle->Instance->CCR1 = pCurrentPwmPin->dutyCycle;
        }
    }
}

#ifdef TIM2
extern void TIM2_IRQHandler(void)
{
#else
extern void TIM3_IRQHandler(void)
{
#endif

    if (pwmCallbackFunc != NULL)
        (*pwmCallbackFunc)();
}
