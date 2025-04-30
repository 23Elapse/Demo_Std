/*
 * @File: rtos_abstraction.c
 * @Description: RTOS抽象层实现
 * @Date: 2025-04-29
 */
#include "stdlib.h"
#include "rtos_abstraction.h"

// RTOS操作实例（需由用户设置）
static const RTOS_Ops_t *rtos_ops = NULL;

int RTOS_SetOps(const RTOS_Ops_t *ops)
{
    if (!ops) return -1;
    rtos_ops = ops;
    return 0;
}

const RTOS_Ops_t* RTOS_GetOps(void)
{
    return rtos_ops;
}
