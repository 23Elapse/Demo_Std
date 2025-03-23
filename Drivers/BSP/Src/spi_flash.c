/* spi_flash.c */
#include "pch.h"
#define flash_timecnt 1000000  // 超时计数器
uint16_t g_norflash_type = BY25Q256;                            /* 默认是BY25Q256 */
static uint8_t sector_buf[SECTOR_SIZE]; /* 扇区缓冲区 */
/* 基础数据传输 */
uint8_t SPI_Flash_Transfer(SPI_Flash_Config* config, uint8_t data)
{
    while(SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(config->SPIx, data);
    
    while(SPI_I2S_GetFlagStatus(config->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(config->SPIx);
}

static void CS_Low(SPI_Flash_Config* config) {
    GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
}

static void CS_High(SPI_Flash_Config* config) {
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
}

/* 发送24位地址 */
static void SendAddress(SPI_Flash_Config* config, uint32_t addr)
{
    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256) /* 只有W25Q256支持4字节地址模式 */
    {
        SPI_Flash_Transfer(config, (uint8_t)((addr)>>24)); /* 发送 bit31 ~ bit24 地址 */
    } 
    SPI_Flash_Transfer(config, (addr >> 16) & 0xFF); // 地址高字节
    SPI_Flash_Transfer(config, (addr >> 8) & 0xFF);  // 地址中字节
    SPI_Flash_Transfer(config, addr & 0xFF);         // 地址低字节
}

/* 写使能操作 */
static Flash_Status WriteEnable(SPI_Flash_Config* config)
{
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_WriteEnable);
    CS_High(config);
    
    /* 验证WEL位是否置位 */
    if((SPI_Flash_ReadStatusReg(config, 1) & 0x02) == 0) {
        return FLASH_WRITE_PROTECTED;
    }
    return FLASH_OK;
}

// static void Flash_Wait_Busy(SPI_Flash_Config* config)
// {
//     while ((SPI_Flash_ReadStatusReg(config, 1) & 0x01) == 0x01);               /* 等待BUSY位清空 */
// }
/* 等待就绪状态 */
static Flash_Status CheckBusy(SPI_Flash_Config* config)
{
    uint32_t timeout = flash_timecnt;
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_ReadStatusReg1);
    
    do {
        uint8_t status = SPI_Flash_Transfer(config, 0xFF);
        if(!(status & STATUS_REG_BUSY)) {
            CS_High(config);
            return FLASH_OK;
        }
    } while(timeout--);
    
    CS_High(config);
    return FLASH_TIMEOUT;
}

/* 定义设备配置 */
SPI_Flash_Config flash_cfg = {
    .SPIx = SPI5,
    .GPIO_Port = GPIOF,
    .CS_Pin = GPIO_Pin_6,
    .SCK_Pin = GPIO_Pin_7,
    .MISO_Pin = GPIO_Pin_8,
    .MOSI_Pin = GPIO_Pin_9,
    .SPI_Clk = RCC_APB2Periph_SPI5,
    .GPIO_Clk = RCC_AHB1Periph_GPIOF,
};


/* 写保护控制 */
Flash_Status SPI_Flash_EnableWriteProtect(SPI_Flash_Config* config, bool enable)
{
    uint8_t status = SPI_Flash_ReadStatusReg(config, 1);
    
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_WriteStatusReg1);
    SPI_Flash_Transfer(config, enable ? (status | 0x80) : (status & ~0x80));
    CS_High(config);
    
    return CheckBusy(config);
}

/* 多状态寄存器读取 */
uint8_t SPI_Flash_ReadStatusReg(SPI_Flash_Config* config, uint8_t regNum)
{
    uint8_t cmd = 0;
    switch(regNum) {
        case 1: cmd = FLASH_ReadStatusReg1; break;
        case 2: cmd = FLASH_ReadStatusReg2; break;
        case 3: cmd = FLASH_ReadStatusReg3; break;
        default: return 0xFF;
    }
    
    CS_Low(config);
    SPI_Flash_Transfer(config, cmd);
    uint8_t status = SPI_Flash_Transfer(config, 0xFF);
    CS_High(config);
    return status;
}
/* 多状态寄存器写入 */
void SPI_Flash_WriteStatusReg(SPI_Flash_Config* config, uint8_t regNum, uint8_t sr)
{
    uint8_t cmd = 0;
    switch(regNum) {
        case 1: cmd = FLASH_WriteStatusReg1; break;
        case 2: cmd = FLASH_WriteStatusReg2; break;
        case 3: cmd = FLASH_WriteStatusReg3; break;
        default: break;
    }
    
    CS_Low(config);
    SPI_Flash_Transfer(config, cmd);
    SPI_Flash_Transfer(config, sr);
    CS_High(config);
}
/* 块保护设置 */
Flash_Status SPI_Flash_SetBlockProtection(SPI_Flash_Config* config, uint8_t protectLevel)
{
    uint8_t status[3] = {0};
    status[0] = SPI_Flash_ReadStatusReg(config, 1);
    status[1] = SPI_Flash_ReadStatusReg(config, 2);
    status[2] = SPI_Flash_ReadStatusReg(config, 3);
    
    /* 修改保护位（根据具体Flash的寄存器布局） */
    status[1] = (status[1] & 0xE3) | ((protectLevel & 0x07) << 2);
    
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_ReadStatusReg1);
    SPI_Flash_Transfer(config, status[0]);
    SPI_Flash_Transfer(config, status[1]);
    SPI_Flash_Transfer(config, status[2]);
    CS_High(config);
    
    return CheckBusy(config);
}
void SPI_Enter4ByteMode(SPI_Flash_Config* config) {
    // 拉低CS引脚
    GPIO_ResetBits(config->GPIO_Port, config->CS_Pin);
    
    // 发送命令0x37（具体指令需参考芯片手册）
    uint8_t cmd = FLASH_Enable4ByteAddr;
    SPI_SendData(config->SPIx, cmd);
    
    // 拉高CS引脚
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin);
}

/* 初始化函数 */
void SPI_Flash_Init(SPI_Flash_Config* config)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(config->GPIO_Clk, ENABLE); 
    if(config->SPIx == SPI2 || config->SPIx == SPI3) {
        RCC_APB1PeriphClockCmd(config->SPI_Clk, ENABLE);
    } else {
        RCC_APB2PeriphClockCmd(config->SPI_Clk, ENABLE);
    }
     /* 配置CS引脚为普通输出 */
    GPIO_InitStruct.GPIO_Pin = config->CS_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);
    GPIO_SetBits(config->GPIO_Port, config->CS_Pin); // 初始拉高CS

    /* 配置SCK/MISO/MOSI复用 */
    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource7, GPIO_AF_SPI5); // SCK
    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource8, GPIO_AF_SPI5); // MISO
    GPIO_PinAFConfig(config->GPIO_Port, GPIO_PinSource9, GPIO_AF_SPI5); // MOSI

    GPIO_InitStruct.GPIO_Pin = config->SCK_Pin | config->MISO_Pin | config->MOSI_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);

     /* 配置CS引脚为普通输出 */
     GPIO_InitStruct.GPIO_Pin = config->CS_Pin;
     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
     GPIO_Init(config->GPIO_Port, &GPIO_InitStruct);
     GPIO_SetBits(config->GPIO_Port, config->CS_Pin); // 初始拉高CS

    /* SPI参数配置 */
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(config->SPIx, &SPI_InitStruct);
    SPI_Cmd(config->SPIx, ENABLE);

    SPI_Enter4ByteMode(config); // 进入4字节地址模式

}

void Flash_init(SPI_Flash_Config* config)
{
    uint8_t temp;
    SPI_Flash_Init(config);
    g_norflash_type = Flash_Read_Id(config);     // 读取flash ID
    printf("flash read id is : %d\r\n", g_norflash_type);
    if (g_norflash_type == W25Q256 || g_norflash_type == BY25Q256)  /* SPI FLASH为25Q256, 必须使能4字节地址模式 */
    {
        temp = SPI_Flash_ReadStatusReg(config, 3);   /* 读取状态寄存器3，判断地址模式 */
        printf("status reg3 is : %d\r\n", temp);
        if ((temp & 0X01) == 0)                                 /* 如果不是4字节地址模式,则进入4字节地址模式 */
        {
            WriteEnable(config);                            /* 写使能 */
            temp |= 1 << 1;                                     /* ADP=1, 上电4位地址模式 */
            SPI_Flash_WriteStatusReg(config, 3, temp);                         /* 写SR3 */
            delay_ms(20);                                       /* 不加延时第一次上电会出错 */
            CS_Low(config);
            SPI_Flash_Transfer(config, FLASH_Enable4ByteAddr);        /* 使能4字节地址指令 */
            CS_High(config);
        }
    }

}

/**
 * @brief       擦除一个扇区
 * @note        注意,这里是字节地址!!（扇区地址 * 4 * 1024）
 *              擦除一个扇区的最少时间:150ms
 * 
 * @param       saddr : 扇区地址 根据实际容量设置
 * @retval      无
 */
Flash_Status SPI_Flash_EraseSector(SPI_Flash_Config* config, uint32_t sectorAddr)
{
    if(config == NULL) return FLASH_INVALID_PARAM;
    if((sectorAddr % (4 * 1024)) != 0) return FLASH_INVALID_PARAM;

    Flash_Status status = WriteEnable(config);
    if(status != FLASH_OK) return status;
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_SectorErase);
    SendAddress(config, sectorAddr);
    CS_High(config);
    return CheckBusy(config);
}

/**
 * @brief       擦除整个芯片
 * @note        等待时间超长...
 * @param       无
 * @retval      无
 */
Flash_Status SPI_Flash_EraseChip(SPI_Flash_Config* config)
{
    if(config == NULL) return FLASH_INVALID_PARAM;

    Flash_Status status = WriteEnable(config);
    if(status != FLASH_OK) return status;
    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_ChipErase);
    CS_High(config);
    return CheckBusy(config);
}

Flash_Status SPI_Flash_WritePage(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
{
    if(config == NULL || pBuffer == NULL || numByteToWrite == 0) 
        return FLASH_INVALID_PARAM;

    Flash_Status status = WriteEnable(config);
    if(status != FLASH_OK) return status;

    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_PageProgram);
    SendAddress(config, writeAddr);
    for(uint16_t i=0; i<numByteToWrite; i++) {
        SPI_Flash_Transfer(config, pBuffer[i]);
    }
    CS_High(config);

    return CheckBusy(config);
}

Flash_Status SPI_Flash_ReadData(SPI_Flash_Config* config, uint8_t* pBuffer, uint32_t readAddr, uint16_t numByteToRead)
{
    if(config == NULL || pBuffer == NULL || numByteToRead == 0) 
        return FLASH_INVALID_PARAM;

    CS_Low(config);
    SPI_Flash_Transfer(config, FLASH_ReadData);
    SendAddress(config, readAddr);
    for(uint16_t i=0; i<numByteToRead; i++) {
        pBuffer[i] = SPI_Flash_Transfer(config, 0xFF);
    }
    CS_High(config);

    return FLASH_OK;
}


uint16_t Flash_Read_Id(SPI_Flash_Config* config)
{
    uint16_t deviceid;

    CS_Low(config);
    SPI_Flash_Transfer(config,FLASH_ManufactDeviceID);   /* 发送读 ID 命令 */
    SPI_Flash_Transfer(config,0);                        /* 写入一个字节 */
    SPI_Flash_Transfer(config,0);
    SPI_Flash_Transfer(config,0);
    deviceid = SPI_Flash_Transfer(config,0xFF) << 8;     /* 读取高8位字节 */
    deviceid |= SPI_Flash_Transfer(config,0xFF);         /* 读取低8位字节 */

    CS_High(config);
    return deviceid;
}

int8_t SPI_Flash_Write_With_Erase(SPI_Flash_Config* config, uint8_t *pbuf, uint32_t addr, uint16_t datalen) 
{
    // 参数检查
    if (!config || !pbuf || datalen == 0) {
        return -1; // 参数错误
    }

    uint32_t start_sector = addr / SECTOR_SIZE;
    uint32_t end_sector = (addr + datalen - 1) / SECTOR_SIZE;

    for (uint32_t sector = start_sector; sector <= end_sector; sector++) {
        uint32_t sector_addr = sector * SECTOR_SIZE;
        uint32_t sector_end = sector_addr + SECTOR_SIZE - 1;

        // 计算当前扇区需要写入的数据范围
        uint32_t data_start = (addr > sector_addr) ? addr : sector_addr;
        uint32_t data_end = (addr + datalen - 1 < sector_end) ? (addr + datalen - 1) : sector_end;
        uint16_t data_len = data_end - data_start + 1;
        uint32_t pbuf_offset = data_start - addr;

        // 完全覆盖整个扇区的优化情况
        if (data_start == sector_addr && data_len == SECTOR_SIZE) {
            // 直接擦除并写入新数据
            SPI_Flash_EraseSector(config, sector_addr);

            uint32_t remain = SECTOR_SIZE;
            uint8_t* src = pbuf + pbuf_offset;
            uint32_t write_addr = sector_addr;

            while (remain > 0) {
                uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                SPI_Flash_WritePage(config, src, write_addr, write_len);
                write_addr += write_len;
                src += write_len;
                remain -= write_len;
            }
        } else {
            // 部分覆盖扇区，需要备份原始数据
            // 读取整个扇区原始数据
            SPI_Flash_ReadData(config, sector_buf, sector_addr, SECTOR_SIZE);

            // 合并新数据到缓冲区
            memcpy(sector_buf + (data_start - sector_addr), pbuf + pbuf_offset, data_len);

            // 擦除扇区
            SPI_Flash_EraseSector(config, sector_addr);

            // 写回整个扇区
            uint32_t remain = SECTOR_SIZE;
            uint8_t* src = sector_buf;
            uint32_t write_addr = sector_addr;

            while (remain > 0) {
                uint16_t write_len = (remain >= PAGE_SIZE) ? PAGE_SIZE : remain;
                SPI_Flash_WritePage(config, src, write_addr, write_len);
                write_addr += write_len;
                src += write_len;
                remain -= write_len;
            }
        }
    }

    return 0; // 成功
}
