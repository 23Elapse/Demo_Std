#include "pch.h"
/**
 * @brief       设置中断向量表偏移地址
 * @param       baseaddr: 基址
 * @param       offset: 偏移量
 * @retval      无
 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
  /* 设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留 */
  SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

/**
 * @brief       执行: WFI指令(执行完该指令进入低功耗状态, 等待中断唤醒)
 * @param       无
 * @retval      无
 */
void sys_wfi_set(void)
{
  __ASM volatile("wfi");
}

/**
 * @brief       关闭所有中断(但是不包括fault和NMI中断)
 * @param       无
 * @retval      无
 */
void sys_intx_disable(void)
{
  __ASM volatile("cpsid i");
}

/**
 * @brief       开启所有中断
 * @param       无
 * @retval      无
 */
void sys_intx_enable(void)
{
  __ASM volatile("cpsie i");
}

/**
 * @brief       设置栈顶地址
 * @note        左侧若出现红X, 属于MDK误报, 实际是没问题的
 * @param       addr: 栈顶地址
 * @retval      无
 */
void sys_msr_msp(uint32_t addr)
{
  __set_MSP(addr); /* 设置栈顶地址 */
}

/**
 * @brief       进入待机模式
 * @param       无
 * @retval      无
 */
void sys_standby(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN; /* 使能电源时钟 */
  SET_BIT(PWR->CR, PWR_CR_PDDS);     /* 进入待机模式 */
}

/**
 * @brief       系统软复位
 * @param       无
 * @retval      无
 */
void sys_soft_reset(void)
{
  NVIC_SystemReset();
}

/**
 * @brief       时钟设置函数
 * @param       plln: PLL1倍频系数(PLL倍频), 取值范围: 64~432.
 * @param       pllm: PLL1预分频系数(进PLL之前的分频), 取值范围: 2~63.
 * @param       pllp: PLL1的p分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 2,4,6,8.(仅限这4个值!)
 * @param       pllq: PLL1的q分频系数(PLL之后的分频), 取值范围: 2~15.
 * @note
 *
 *              Fvco: VCO频率
 *              Fsys: 系统时钟频率, 也是PLL1的p分频输出时钟频率
 *              Fq:   PLL1的q分频输出时钟频率
 *              Fs:   PLL输入时钟频率, 可以是HSI, HSE等.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              外部晶振为25M的时候, 推荐值: plln = 360, pllm = 25, pllp = 2, pllq = 8.
 *              得到:Fvco = 25 * (360 / 25) = 360Mhz
 *                   Fsys = pll1_p_ck = 360 / 2 = 180Mhz
 *                   Fq   = pll1_q_ck = 360 / 8 = 45(使用USB时，需设置plln=384，即可得到48Mhz频率, 此时主频为192M)
 *
 *              F429默认需要配置的频率如下:
 *              CPU频率(HCLK) = pll_p_ck = 180Mhz
 *              AHB1/2/3(rcc_hclk1/2/3) = 180Mhz
 *              APB1(rcc_pclk1) = pll_p_ck / 4 = 45Mhz
 *              APB2(rcc_pclk2) = pll_p_ck / 2 = 90Mhz
 *
 * @retval      错误代码: 0, 成功; 1, 错误;
 */

/**
 * @brief 时钟设置函数
 * @param plln: PLL1倍频系数(PLL倍频), 取值范围: 64~432.
 * @param pllm: PLL1预分频系数(进PLL之前的分频), 取值范围: 2~63.
 * @param pllp: PLL1的p分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 2,4,6,8.
 * @param pllq: PLL1的q分频系数(PLL之后的分频), 取值范围: 2~15.
 * @retval 错误代码: 0, 成功; 1, 错误;
 */
// 确保SystemCoreClock已更新为实际SYSCLK值
extern uint32_t SystemCoreClock;
uint8_t SystemClock_Config(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
  RCC_DeInit();

  // 使能HSE
  RCC_HSEConfig(RCC_HSE_ON);
  if (RCC_WaitForHSEStartUp() != SUCCESS)
    return 1;

  // 参数有效性检查
  if ((pllm < 2) || (pllm > 63))
    return 1; // PLLM范围2~63
  if ((plln < 192) || (plln > 432))
    return 1; // PLLN范围192~432
  if ((pllp != 2) && (pllp != 4) && (pllp != 6) && (pllp != 8))
    return 1; // PLLP只能取2,4,6,8
  if ((pllq < 2) || (pllq > 15))
    return 1; // PLLQ范围2~15

  // 假设HSE频率为25MHz，计算VCO输入频率
  uint32_t HSE_Freq = 25000000; // 根据实际硬件调整
  float VCO_input = (float)HSE_Freq / pllm;
  if (VCO_input < 1.0f || VCO_input > 2.0f)
    return 1; // VCO输入需在1~2MHz

  // 计算VCO输出频率及系统时钟
  float VCO_output = VCO_input * plln;
  if (VCO_output < 192e6f || VCO_output > 432e6f)
    return 1; // VCO输出需在192~432MHz
  float SysClock = VCO_output / pllp;
  if (SysClock > 180e6f)
    return 1; // 系统时钟不得超过180MHz

  // 配置PLL
  RCC_PLLConfig(RCC_PLLSource_HSE, pllm, plln, pllp, pllq);
  RCC_PLLCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    ;

  // 设置Flash等待周期（根据SysClock）
  if (SysClock <= 30e6f)
    FLASH_SetLatency(FLASH_Latency_0);
  else if (SysClock <= 60e6f)
    FLASH_SetLatency(FLASH_Latency_1);
  else if (SysClock <= 90e6f)
    FLASH_SetLatency(FLASH_Latency_2);
  else if (SysClock <= 120e6f)
    FLASH_SetLatency(FLASH_Latency_3);
  else if (SysClock <= 150e6f)
    FLASH_SetLatency(FLASH_Latency_4);
  else
    FLASH_SetLatency(FLASH_Latency_5);
  FLASH_PrefetchBufferCmd(ENABLE);

  // 配置总线分频（切换时钟源前设置）
  RCC_HCLKConfig(RCC_SYSCLK_Div1); // HCLK = SysClock
  RCC_PCLK1Config(RCC_HCLK_Div4);  // APB1 = HCLK/4 (45MHz @180MHz)
  RCC_PCLK2Config(RCC_HCLK_Div2);  // APB2 = HCLK/2 (90MHz @180MHz)

  // 切换系统时钟源到PLL
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while (RCC_GetSYSCLKSource() != 0x08)
    ;
  SystemCoreClockUpdate();  // 更新SystemCoreClock为当前SYSCLK值
  return 0;
}
/*
 * 使用HSE时，设置系统时钟的步骤
 * 1、开启HSE ，并等待 HSE 稳定
 * 2、设置 AHB、APB2、APB1的预分频因子
 * 3、设置PLL的时钟来源
 *    设置VCO输入时钟 分频因子        m
 *    设置VCO输出时钟 倍频因子        n
 *    设置PLLCLK时钟分频因子          p
 *    设置OTG FS,SDIO,RNG时钟分频因子 q
 * 4、开启PLL，并等待PLL稳定
 * 5、把PLLCK切换为系统时钟SYSCLK
 * 6、读取时钟切换状态位，确保PLLCLK被选为系统时钟
 */

/*
 * m: VCO输入时钟 分频因子，取值2~63
 * n: VCO输出时钟 倍频因子，取值192~432
 * p: PLLCLK时钟分频因子  ，取值2，4，6，8
 * q: OTG FS,SDIO,RNG时钟分频因子，取值4~15
 * 函数调用举例，使用HSE设置时钟
 * SYSCLK=HCLK=180M,PCLK2=HCLK/2=90M,PCLK1=HCLK/4=45M
 * HSE_SetSysClock(25, 360, 2, 7);
 * HSE作为时钟来源，经过PLL倍频作为系统时钟，这是通常的做法

 * 系统时钟超频到216M爽一下
 * HSE_SetSysClock(25, 432, 2, 9);
 */
void HSE_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
  __IO uint32_t HSEStartUpStatus = 0;

  // 使能HSE，开启外部晶振，野火F429使用 HSE=25M
  RCC_HSEConfig(RCC_HSE_ON);

  // 等待HSE启动稳定
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    // 调压器电压输出级别配置为1，以便在器件为最大频率
    // 工作时使性能和功耗实现平衡
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // HCLK = SYSCLK / 1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    // PCLK2 = HCLK / 2
    RCC_PCLK2Config(RCC_HCLK_Div2);

    // PCLK1 = HCLK / 4
    RCC_PCLK1Config(RCC_HCLK_Div4);

    // 如果要超频就得在这里下手啦
    // 设置PLL来源时钟，设置VCO分频因子m，设置VCO倍频因子n，
    //  设置系统时钟分频因子p，设置OTG FS,SDIO,RNG分频因子q
    RCC_PLLConfig(RCC_PLLSource_HSE, m, n, p, q);

    // 使能PLL
    RCC_PLLCmd(ENABLE);

    // 等待 PLL稳定
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /*-----------------------------------------------------*/
    // 开启 OVER-RIDE模式，以能达到更高频率
    PWR->CR |= PWR_CR_ODEN;
    while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
    {
    }
    PWR->CR |= PWR_CR_ODSWEN;
    while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
    {
    }
    // 配置FLASH预取指,指令缓存,数据缓存和等待状态
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
    /*-----------------------------------------------------*/

    // 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // 读取时钟切换状态位，确保PLLCLK被选为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { // HSE启动出错处理

    while (1)
    {
    }
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief       当编译提示出错的时候此函数用来报告错误的文件和所在行
 * @param       file：指向源文件
 * @param       line：指向在文件中的行数
 * @retval      无
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}

#endif
