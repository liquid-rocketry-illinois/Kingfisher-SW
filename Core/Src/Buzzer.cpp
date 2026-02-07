//
// Created by dyrel on 2/7/2026.
//

#include "Buzzer.h"

typedef enum {
    BUZZER_STATE_DIGITAL,
    BUZZER_STATE_PWM
} BuzzerState_t;

static BuzzerState_t buzzerState = BUZZER_STATE_DIGITAL;

void buzzerON()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;  // adjust if needed

    HAL_GPIO_DeInit(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    buzzerState = BUZZER_STATE_PWM;
}

void buzzerOFF()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

    GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_DeInit(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    buzzerState = BUZZER_STATE_DIGITAL;
}

void buzzerTOGGLE()
{
    if (buzzerState == BUZZER_STATE_DIGITAL)
    {
        buzzerON();   // switch to PWM
    }
    else
    {
        buzzerOFF();  // switch to digital HIGH
    }
}
