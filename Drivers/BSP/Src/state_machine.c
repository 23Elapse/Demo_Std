#include "state_machine.h"
#include "pch.h"
void StateMachine_Init(StateContext_t *ctx, void *user_data)
{
    ctx->current_state = STATE_INIT;
    ctx->index = 0;
    ctx->expected_length = 0;
    ctx->user_data = user_data;
}

void StateMachine_Process(StateContext_t *ctx, StateTransition_t *transitions, uint8_t byte)
{
    for (int i = 0; transitions[i].handler != NULL; i++)
    {
        if (transitions[i].state == ctx->current_state)
        {
            if (transitions[i].handler(ctx, byte, ctx->user_data))
            {
                break;
            }
        }
    }
}
