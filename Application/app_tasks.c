/**
 * =====================================================================================
 * @file        app_tasks.c
 * @brief       应用层任务创建与系统初始化协调中心 (Refactored)
 * @author      23Elapse & Gemini
 * @version     2.1 (Refactored)
 * @date        2025-06-14
 * @note        这是系统启动的总入口，负责按顺序初始化所有模块并创建任务。
 * =====================================================================================
 */
#include "app_tasks.h"
#include "dev_config.h"         // 关键：包含所有设备实例声明
#include "device_manager.h"     // 关键：包含设备管理器接口 (可选，如果无需通用管理)
#include "rtos_abstraction.h"   // RTOS 抽象层
#include "log_system.h"         // 日志系统
#include "serial_interface.h"   // 串口中间层接口
#include "can_driver.h"         // CAN 驱动接口
#include "i2c_driver.h"         // I2C 总线驱动接口
#include "spi_flash.h"          // SPI Flash 驱动接口
#include "pcf8574.h"            // PCF8574 驱动接口 (如果作为独立设备管理)
#include "tsk_eeprom.h"         // EEPROM 任务 (假设它有自己的初始化和任务创建)
#include "FreeRTOS.h"           // FreeRTOS 核心
#include "task.h"               // FreeRTOS 任务API
#include "stm32f4xx_can.h"
#include "rs485_driver.h"
#include "tsk_wifi.h"

/*
 * =====================================================================================
 * 宏定义 - 任务参数
 * =====================================================================================
 */
#define TASK_RS485_POLL_STK_SIZE    256
#define TASK_RS485_POLL_PRIO        2    //值越大优先级越高

#define TASK_SERIAL_RX_STK_SIZE     512 // 接收任务栈可能需要大一些，处理协议解析
#define TASK_SERIAL_RX_PRIO         3

#define TASK_ERROR_LOG_STK_SIZE     128
#define TASK_ERROR_LOG_PRIO         1

#define TASK_WIFI_BLE_STK_SIZE      1024 // WiFi/BLE任务需要较大栈，特别是处理AT命令
#define TASK_WIFI_BLE_PRIO          2

#define TASK_CAN_STK_SIZE           256
#define TASK_CAN_PRIO               3

#define TASK_SPI_FLASH_STK_SIZE     512 // Flash操作可能需要较大栈和缓冲区
#define TASK_SPI_FLASH_PRIO         2

#define TASK_I2C_STK_SIZE           256
#define TASK_I2C_PRIO               2

#define TASK_WATCHDOG_STK_SIZE      128
#define TASK_WATCHDOG_PRIO          1

/*
 * =====================================================================================
 * 模块私有变量
 * =====================================================================================
 */
// 设备管理器 (可选，如果无需通用设备注册/查找功能，可以移除)
#define MAX_DEVICES 10
static Device_Handle_t g_device_array[MAX_DEVICES];
static Device_Manager_t g_device_mgr;

// PCF8574 设备实例（需要在这里定义，因为 dev_config.h 中只声明了总线）
static PCF8574_Device_t g_pcf8574_dev = {
    .i2c_bus = &g_i2c1_bus,       // 指向 dev_config.c 中定义的 I2C 总线
    .dev_addr = PCF8574_DEFAULT_ADDR, // 使用默认地址
    .current_io_state = 0x00      // 初始IO状态
};

/*
 * =====================================================================================
 * 内部函数声明
 * =====================================================================================
 */
static void App_RTOS_Init(void);
static void App_Device_Register(void); // 如果使用 Device Manager
static void App_Driver_Init(void);
static void App_Tasks_Create(void);
static void App_RS485_PollTask(void* param);
static void App_SerialRxTask(void* param);
static void App_ErrorLogTask(void* param);
static void App_WifiBLETask(void* param);
// static void App_CAN_Task(void* param);
static void App_SPIFlashTask(void* param);
static void App_I2CTask(void* param);
static void App_WatchDogTask(void* param);

/*
 * =====================================================================================
 * 主初始化函数 (系统入口)
 * =====================================================================================
 */
void App_Init(void) {
    App_RTOS_Init();          // 1. 初始化RTOS抽象层
    App_Device_Register();    // 2. 注册设备到设备管理器 (可选)
    App_Driver_Init();        // 3. 初始化所有硬件驱动
    App_Tasks_Create();       // 4. 创建所有应用任务

    Log_Message(LOG_LEVEL_INFO, "[App] Starting RTOS scheduler...");
    g_rtos_ops->TaskStartScheduler(); // 启动RTOS调度器

    // 如果调度器退出，则表示发生严重错误
    Log_Message(LOG_LEVEL_ERROR, "FATAL: Scheduler exited unexpectedly!");
    for (;;); // 停止在此
}

/*
 * =====================================================================================
 * 初始化辅助函数实现
 * =====================================================================================
 */

/**
 * @brief 初始化RTOS抽象层
 */
static void App_RTOS_Init(void) {
    g_rtos_ops = &FreeRTOS_Ops; // 设置全局RTOS操作接口为FreeRTOS实现
    if (!g_rtos_ops || !g_rtos_ops->SemaphoreCreate || !g_rtos_ops->TaskCreate) {
        // 如果RTOS抽象层未正确设置，则系统无法运行
        // 在实际项目中，这里可能需要一个LED闪烁来指示致命错误
        Log_Message(LOG_LEVEL_ERROR, "[App] RTOS abstraction layer not properly initialized!");
        for (;;); // 停机
    }
    Log_Message(LOG_LEVEL_INFO, "[App] RTOS abstraction initialized.");
}

/**
 * @brief 注册设备到设备管理器
 * @note  如果不需要通用设备管理功能，此函数及其调用可以移除。
 */
static void App_Device_Register(void) {
    DeviceManager_Init(&g_device_mgr, g_device_array, MAX_DEVICES); // 初始化设备管理器
    Log_Message(LOG_LEVEL_INFO, "[App] Registering devices to manager...");

    // 注册 dev_config.h 中声明的全局设备实例
    DeviceManager_Register(&g_device_mgr, &g_rs485_serial, DEVICE_TYPE_SERIAL, 1);
    DeviceManager_Register(&g_device_mgr, &g_uart_dev,     DEVICE_TYPE_SERIAL, 2);
    DeviceManager_Register(&g_device_mgr, &g_esp32_serial,  DEVICE_TYPE_SERIAL, 3);
    DeviceManager_Register(&g_device_mgr, &g_can1_dev,      DEVICE_TYPE_CAN_BUS, 1);
    DeviceManager_Register(&g_device_mgr, &g_i2c1_bus,      DEVICE_TYPE_I2C_BUS, 1);
    DeviceManager_Register(&g_device_mgr, &g_spi_flash_dev, DEVICE_TYPE_SPI_FLASH, 1);
    DeviceManager_Register(&g_device_mgr, &g_esp32_dev,     DEVICE_TYPE_ESP32, 1);
    DeviceManager_Register(&g_device_mgr, &g_pcf8574_dev,   DEVICE_TYPE_I2C_SLAVE, 1); // 注册PCF8574设备
    Log_Message(LOG_LEVEL_INFO, "[App] All devices registered.");
}

/**
 * @brief 初始化所有硬件驱动
 */
static void App_Driver_Init(void) {
    Log_Message(LOG_LEVEL_INFO, "[App] Initializing hardware drivers...");

    // 初始化串口驱动 (底层)
    g_serial_ops.Init(&g_rs485_serial);
    g_serial_ops.Init(&g_uart_dev);
    g_serial_ops.Init(&g_esp32_serial);

    // 初始化 CAN 驱动
    g_can_ops.Init(&g_can1_dev);

    // 初始化 I2C 总线驱动
    g_i2c_bus_ops.Init(&g_i2c1_bus);

    // 初始化 SPI Flash 驱动
    SPI_Flash_Device_Init(&g_spi_flash_dev);

    // 初始化 ESP32 硬件 (复位引脚等)
    ESP32_Hw_Init(&g_esp32_dev);

    // 初始化 PCF8574 驱动 (作为I2C从设备)
//    PCF8574_Init(&g_pcf8574_dev);

    // 初始化高层服务/模块 (例如 EEPROM 任务)
    // Tsk_Eeprom_Init(); // 假设此函数包含其自身任务的创建
    Log_Message(LOG_LEVEL_INFO, "[App] All hardware drivers initialized.");
}

/**
 * @brief 创建所有应用任务
 */
static void App_Tasks_Create(void) {
    Log_Message(LOG_LEVEL_INFO, "[App] Creating application tasks...");

    // RS485 轮询发送任务 (负责从队列中取出帧并通过g_rs485_serial发送)
    g_rtos_ops->TaskCreate(App_RS485_PollTask, "RS485_Poll", TASK_RS485_POLL_STK_SIZE, &g_rs485_serial, TASK_RS485_POLL_PRIO);

    // 串口接收任务 (处理通用串口数据和RS485协议解析，这里以g_esp32_serial为例)
    // g_rtos_ops->TaskCreate(App_SerialRxTask, "Serial_Rx", TASK_SERIAL_RX_STK_SIZE, &g_esp32_serial, TASK_SERIAL_RX_PRIO);

    // 错误日志处理任务
    g_rtos_ops->TaskCreate(App_ErrorLogTask, "Error_Log", TASK_ERROR_LOG_STK_SIZE, NULL, TASK_ERROR_LOG_PRIO);

    // WiFi/BLE 统一管理任务
    g_rtos_ops->TaskCreate(App_WifiBLETask, "WiFi_BLE", TASK_WIFI_BLE_STK_SIZE, NULL, TASK_WIFI_BLE_PRIO);

    // CAN 通信任务
    // g_rtos_ops->TaskCreate(App_CANTask, "CAN", TASK_CAN_STK_SIZE, &g_can1_dev, TASK_CAN_PRIO);

    // SPI Flash 管理任务
    g_rtos_ops->TaskCreate(App_SPIFlashTask, "SPI_Flash", TASK_SPI_FLASH_STK_SIZE, &g_spi_flash_dev, TASK_SPI_FLASH_PRIO);

    // watch dog task
    // g_rtos_ops->TaskCreate(App_WatchDogTask, "WatchDog", TASK_WATCHDOG_STK_SIZE, NULL, TASK_WATCHDOG_PRIO);

    // I2C 测试任务 (PCF8574)
    // g_rtos_ops->TaskCreate(App_I2CTask, "I2C_Test", TASK_I2C_STK_SIZE, &g_pcf8574_dev, TASK_I2C_PRIO);

    // 注意：EepromMonitorTask (或其他模块任务) 由其各自的 Init 函数内部创建
    Log_Message(LOG_LEVEL_INFO, "[App] All application tasks created.");
}

/*
 * =====================================================================================
 * 任务函数实现
 * =====================================================================================
 */

/** * @brief 看门狗任务 (定期喂狗)
 * @param pvParameters 指向任务参数 (如果需要，可以传入设备实例或其他数据)
 */

void App_WatchDogTask(void *pvParameters) {
    Log_Message(LOG_LEVEL_TEST, "[WatchDog] Task started.");
    while (1) {
        // 这里可以添加实际的看门狗逻辑，例如定期喂狗
        Log_Message(LOG_LEVEL_TEST, "[SysTick] CTRL=0x%08lx LOAD=%lu VAL=%lu", SysTick->CTRL, SysTick->LOAD, SysTick->VAL);    

        Log_Message(LOG_LEVEL_TEST, "[WatchDog] Feeding the watchdog...");
        g_rtos_ops->Delay(1000); // 每秒喂一次狗
    }
}
/**
 * @brief RS485 轮询发送任务
 * @param pvParameters 指向 Serial_Device_t 实例 (应为 RS485 模式)
 */
void App_RS485_PollTask(void *pvParameters) {
    Serial_Device_t *rs485_serial_dev = (Serial_Device_t *)pvParameters;
    if (!rs485_serial_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[RS485 Poll] Invalid device parameter. Suspending task.");
        if (g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[RS485 Poll] Task started for USART %p.", rs485_serial_dev->instance);

    // 封装为 RS485_Device_t 便于使用 g_rs485_ops
    RS485_Device_t rs485_dev = {.serial_dev = rs485_serial_dev};
    g_rs485_ops.Init(&rs485_dev); // 初始化 RS485 逻辑层

    while (1) {
        g_rs485_ops.PollSendQueue(&rs485_dev); // 轮询发送队列
        g_rtos_ops->Delay(10); // 轮询间隔，可调整
    }
}



/**
 * @brief 错误日志任务
 */
void App_ErrorLogTask(void *pvParameters) {
    Log_Message(LOG_LEVEL_INFO, "[Error Log] Task started.");
    Serial_ErrorLog_t log_entry;
    while (1) {
        // 阻塞等待错误日志，直到有日志可用 (0xFFFFFFFF 表示永远等待)
        if (g_serial_ops.GetErrorLog(&log_entry, 0xFFFFFFFF) == SERIAL_OK) {
            // 打印错误日志
            Log_Message(LOG_LEVEL_WARNING, "[ErrorLog] Type=%u, Instance=%p, Timestamp=%lu",
                        (unsigned int)log_entry.type, log_entry.instance, (unsigned long)log_entry.timestamp);
        }
    }
}



/**
 * @brief CAN 管理任务
 * @param pvParameters 指向 CAN_Device_t 实例
 */
void App_CANTask(void *pvParameters) {
    CAN_Device_t *can_dev = (CAN_Device_t *)pvParameters;
    if (!can_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[CAN] Invalid device parameter. Suspending task.");
        if (g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[CAN Task] Started for CAN instance %p.", can_dev->instance);

    uint32_t msg_id_counter = 0; // 计数器，用于生成不同的消息ID

    while (1) {
        // 构造要发送的CAN消息
        CAN_Message_t tx_msg = {
            .id = 0x100 + (msg_id_counter % 0x10), // 消息ID递增
            .length = 8,
            .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, (uint8_t)msg_id_counter}, // 数据也递增
            .ide = CAN_Id_Standard, // 标准帧
            .rtr = CAN_RTR_Data     // 数据帧
        };

        // 发送CAN消息
        if (g_can_ops.SendMessage(can_dev, &tx_msg) == CAN_OK) {
            Log_Message(LOG_LEVEL_DEBUG, "[CAN] Sent message ID: 0x%03X, Data[7]=0x%02X.", tx_msg.id, tx_msg.data[7]);
            msg_id_counter++;
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[CAN] Failed to send message from CAN %p.", can_dev->instance);
        }

        // 尝试接收CAN消息
        CAN_Message_t rx_msg;
        if (g_can_ops.ReceiveMessage(can_dev, &rx_msg, 50) == CAN_OK) { // 50ms超时
            Log_Message(LOG_LEVEL_INFO, "[CAN] Recv ID: 0x%03X, Len: %u, Data: %02X %02X %02X %02X ...",
                        rx_msg.id, rx_msg.length, rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3]);
        } else {
            // Log_Message(LOG_LEVEL_DEBUG, "[CAN] No message received from CAN %p within timeout.", can_dev->instance);
        }
        g_rtos_ops->Delay(100); // 任务延时
    }
}

/**
 * @brief SPI Flash 管理任务
 * @param pvParameters 指向 SPI_Flash_Device_t 实例
 */
void App_SPIFlashTask(void *pvParameters) {
    SPI_Flash_Device_t *flash_dev = (SPI_Flash_Device_t *)pvParameters;
    if (!flash_dev || !flash_dev->config) {
        Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Invalid device parameter. Suspending task.");
        if (g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[SPI Flash Task] Started.");

    uint8_t test_val_counter = 0;
    uint32_t test_address = 0x000000; // 测试写入地址

    while (1) {
        uint8_t write_buffer[PAGE_SIZE]; // 使用PAGE_SIZE确保能写入一整页
        // 填充写入缓冲区
        for (uint16_t i = 0; i < PAGE_SIZE; i++) {
            write_buffer[i] = test_val_counter + (i % 256);
        }
        
        // 使用带擦除的写入操作
        Flash_Status_t write_status = SPI_Flash_WriteWithErase(flash_dev, write_buffer, test_address, PAGE_SIZE);

        if (write_status == FLASH_OK) {
            uint8_t read_buffer[PAGE_SIZE];
            Flash_Status_t read_status = SPI_Flash_ReadData(flash_dev, read_buffer, test_address, PAGE_SIZE);

            if (read_status == FLASH_OK) {
                if (memcmp(write_buffer, read_buffer, PAGE_SIZE) == 0) {
                    Log_Message(LOG_LEVEL_INFO, "[SPI Flash] R/W Test OK. Addr: 0x%08lX, Data[0]=0x%02X.", test_address, read_buffer[0]);
                } else {
                    Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] R/W Mismatch! Addr: 0x%08lX.", test_address);
                    // 可以打印出不匹配的部分，帮助调试
                }
            } else {
                Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Read failed after write. Status: %d.", read_status);
            }
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[SPI Flash] Write with erase failed. Status: %d.", write_status);
        }

        test_val_counter++; // 更新测试值
        // test_address = (test_address + PAGE_SIZE) % (16 * SECTOR_SIZE); // 移动到下一个地址进行测试
        g_rtos_ops->Delay(5000); // 延时5秒
    }
}

/**
 * @brief I2C 测试任务 (使用PCF8574进行IO控制)
 * @param pvParameters 指向 PCF8574_Device_t 实例
 */
void App_I2CTask(void *pvParameters) {
    PCF8574_Device_t *pcf8574_dev = (PCF8574_Device_t *)pvParameters;
    if (!pcf8574_dev) {
        Log_Message(LOG_LEVEL_ERROR, "[I2C Task] Invalid PCF8574 device parameter. Suspending task.");
        if (g_rtos_ops && g_rtos_ops->Task_Suspend) g_rtos_ops->Task_Suspend(NULL); else for(;;);
    }
    Log_Message(LOG_LEVEL_INFO, "[I2C Task] Started for PCF8574 on I2C bus %p.", pcf8574_dev->i2c_bus->scl_port);

    uint8_t io_state = 0x01; // 初始IO状态，逐位点亮

    while (1) {
        // 1. 写入PCF8574，控制IO口 (例如点亮一个LED)
        I2C_Status_t write_status = PCF8574_WriteByte(pcf8574_dev, io_state);
        if (write_status == I2C_OK) {
            Log_Message(LOG_LEVEL_INFO, "[I2C] PCF8574 Write OK: 0x%02X.", io_state);
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[I2C] PCF8574 Write Failed. Status: %d.", write_status);
        }

        g_rtos_ops->Delay(500); // 延时500ms

        // 2. 从PCF8574读取IO口状态 (例如读取按键输入)
        uint8_t read_data = 0;
        I2C_Status_t read_status = PCF8574_ReadByte(pcf8574_dev, &read_data);
        if (read_status == I2C_OK) {
            Log_Message(LOG_LEVEL_INFO, "[I2C] PCF8574 Read OK: 0x%02X.", read_data);
            // 这里可以根据读取到的数据做进一步处理，例如判断按键是否按下
        } else {
            Log_Message(LOG_LEVEL_ERROR, "[I2C] PCF8574 Read Failed. Status: %d.", read_status);
        }

        // 循环切换IO状态
        io_state = (io_state << 1) | ((io_state >> 7) & 0x01); // 循环左移，实现流水灯效果
        if (io_state == 0) io_state = 0x01; // 防止变为0导致所有灯灭

        g_rtos_ops->Delay(500); // 延时500ms
    }
}


/*
 * =====================================================================================
 * FreeRTOS 钩子函数 (通常放在 app_tasks.c 的末尾)
 * =====================================================================================
 */

/**
 * @brief  当 pvPortMalloc() 返回 NULL 时，此钩子函数会被调用。
 * @note   通常是因为FreeRTOS的堆空间不足。
 */
void vApplicationMallocFailedHook(void)
{
    // 在这里设置一个断点！
    // 如果程序停在这里，就说明是总堆空间(configTOTAL_HEAP_SIZE)不足导致的。
    Log_Message(LOG_LEVEL_ERROR, "FATAL: FreeRTOS Malloc Failed! Heap size might be too small.");
    taskDISABLE_INTERRUPTS(); // 禁用所有中断
    for(;;) // 进入死循环
    {
    }
}

/**
 * @brief  当检测到任务栈溢出时，此钩子函数会被调用。
 * @param  pxTask: 发生溢出的任务句柄
 * @param  pcTaskName: 发生溢出的任务名
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask; // 未使用参数

    // 在这里设置一个断点！
    // 如果程序停在这里，就说明名为 pcTaskName 的任务发生了栈溢出。
    // 你需要增加创建该任务时分配的堆栈大小。
    Log_Message(LOG_LEVEL_ERROR, "FATAL: Stack overflow in task: %s", pcTaskName);
    taskDISABLE_INTERRUPTS(); // 禁用所有中断
    for(;;) // 进入死循环
    {
    }
}
