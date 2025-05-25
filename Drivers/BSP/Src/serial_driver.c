#include "serial_driver.h"
#include "pch.h"
#include "rtos_abstraction.h"

static Serial_Device_t *serial_devices[4] = {0};
static uint8_t serial_device_count = 0;

static Serial_Status Serial_GPIO_Init(Serial_Device_t *dev)
{
    if (!dev->tx_port || !dev->rx_port || (dev->mode == RS485_MODE && !dev->de_port))
    {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->tx_port, dev->tx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }

    if (Common_GPIO_Init(dev->rx_port, dev->rx_pin, GPIO_Mode_AF, GPIO_OType_PP,
                         GPIO_PuPd_UP, GPIO_Speed_50MHz, dev->af) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }

    if (dev->mode == RS485_MODE)
    {
        if (Common_GPIO_Init(dev->de_port, dev->de_pin, GPIO_Mode_OUT, GPIO_OType_PP,
                             GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, 0) != COMMON_OK)
        {
            return SERIAL_ERR_INIT;
        }
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    return SERIAL_OK;
}

static Serial_Status Serial_USART_Init(Serial_Device_t *dev)
{
    if (!dev->instance)
    {
        return SERIAL_ERR_INIT;
    }

    if (Common_USART_Init(dev->instance, dev->baudrate, USART_WordLength_8b,
                          USART_StopBits_1, USART_Parity_No) != COMMON_OK)
    {
        return SERIAL_ERR_INIT;
    }

    return SERIAL_OK;
}

static Serial_Status Serial_NVIC_Init(Serial_Device_t *dev)
{
    if (!dev->instance)
    {
        return SERIAL_ERR_INIT;
    }

    USART_ITConfig(dev->instance, USART_IT_RXNE, ENABLE);
    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = dev->irqn,
        .NVIC_IRQChannelPreemptionPriority = 5,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE};
    NVIC_Init(&nvic_init);
    return SERIAL_OK;
}

static Serial_Status Serial_RingBuffer_Init(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops)
        return SERIAL_ERR_INIT;
    if (RingBuffer_Init(&dev->rx_buffer, 16, sizeof(uint8_t)) != RB_OK)
    {
        return SERIAL_ERR_INIT;
    }
    return SERIAL_OK;
}

Serial_Status Serial_Driver_Init(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev || !dev->instance || serial_device_count >= 4)
    {
        return SERIAL_ERR_INIT;
    }

    Serial_Status status;

    status = Serial_GPIO_Init(dev);
    if (status != SERIAL_OK)
    {
        Serial_Driver_Deinit(dev);
        return status;
    }

    status = Serial_USART_Init(dev);
    if (status != SERIAL_OK)
    {
        Serial_Driver_Deinit(dev);
        return status;
    }

    status = Serial_NVIC_Init(dev);
    if (status != SERIAL_OK)
    {
        Serial_Driver_Deinit(dev);
        return status;
    }

    status = Serial_RingBuffer_Init(dev);
    if (status != SERIAL_OK)
    {
        Serial_Driver_Deinit(dev);
        return status;
    }

    uint32_t silent_us = (35 * 1000000) / dev->baudrate;
    dev->silent_ticks = silent_us / 1000;

    serial_devices[serial_device_count++] = dev;
    return SERIAL_OK;
}

Serial_Status Serial_Driver_Deinit(Serial_Device_t *dev)
{
    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (!rtos_ops || !dev)
        return SERIAL_ERR_INIT;

    if (dev->instance)
    {
        USART_ITConfig(dev->instance, USART_IT_RXNE, DISABLE);
        USART_Cmd(dev->instance, DISABLE);
    }

    if (RingBuffer_Deinit(&dev->rx_buffer) != RB_OK)
    {
        return SERIAL_ERR_INIT;
    }

    for (uint8_t i = 0; i < serial_device_count; i++)
    {
        if (serial_devices[i] == dev)
        {
            serial_devices[i] = serial_devices[--serial_device_count];
            serial_devices[serial_device_count] = NULL;
            break;
        }
    }

    return SERIAL_OK;
}

Serial_Status Serial_Driver_SendData(Serial_Device_t *dev, const uint8_t *data, uint32_t length)
{
    if (!dev || !data)
        return SERIAL_ERR_INIT;

    if (dev->mode == RS485_MODE)
    {
        GPIO_SetBits(dev->de_port, dev->de_pin);
    }

    uint32_t timeout = 1000000;
    for (uint32_t i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(dev->instance, USART_FLAG_TXE) == RESET && timeout--)
        {
        }
        if (timeout == 0)
        {
            if (dev->mode == RS485_MODE)
            {
                GPIO_ResetBits(dev->de_port, dev->de_pin);
            }
            return SERIAL_ERR_TIMEOUT;
        }
        USART_SendData(dev->instance, data[i]);
    }

    timeout = 1000000;
    while (USART_GetFlagStatus(dev->instance, USART_FLAG_TC) == RESET && timeout--)
    {
    }
    if (timeout == 0)
    {
        if (dev->mode == RS485_MODE)
        {
            GPIO_ResetBits(dev->de_port, dev->de_pin);
        }
        return SERIAL_ERR_TIMEOUT;
    }

    if (dev->mode == RS485_MODE)
    {
        GPIO_ResetBits(dev->de_port, dev->de_pin);
    }

    return SERIAL_OK;
}

Serial_Status Serial_Driver_ReceiveByte(Serial_Device_t *dev, uint8_t *byte, void *xHigherPriorityTaskWoken)
{
    if (!dev || !byte)
        return SERIAL_ERR_INIT;

    if (USART_GetITStatus(dev->instance, USART_IT_RXNE) != RESET)
    {
        *byte = USART_ReceiveData(dev->instance);
        USART_ClearITPendingBit(dev->instance, USART_IT_RXNE);
        return SERIAL_OK;
    }

    return SERIAL_ERR_NO_DATA;
}

void Serial_Driver_IRQHandler(Serial_Device_t *dev)
{
    void *xHigherPriorityTaskWoken = NULL;
    uint8_t byte;

    if (Serial_Driver_ReceiveByte(dev, &byte, xHigherPriorityTaskWoken) == SERIAL_OK)
    {
        RingBuffer_WriteFromISR(&dev->rx_buffer, &byte, xHigherPriorityTaskWoken);
    }

    const RTOS_Ops_t *rtos_ops = RTOS_GetOps();
    if (rtos_ops)
    {
        rtos_ops->YieldFromISR(xHigherPriorityTaskWoken);
    }
}
