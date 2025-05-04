#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "stdint.h"

typedef enum
{
    STATE_INIT,
    STATE_START,
    STATE_DATA,
    STATE_END
} State_t;

typedef struct
{
    State_t current_state;
    uint32_t index;
    uint8_t expected_length;
    void *user_data;
} StateContext_t;

typedef int (*StateHandler_t)(StateContext_t *ctx, uint8_t byte, void *user_data);

typedef struct
{
    State_t state;
    StateHandler_t handler;
} StateTransition_t;

void StateMachine_Init(StateContext_t *ctx, void *user_data);
void StateMachine_Process(StateContext_t *ctx, StateTransition_t *transitions, uint8_t byte);

#endif /* __STATE_MACHINE_H */
