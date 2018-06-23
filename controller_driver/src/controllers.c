#include <controllers.h>
#include <common.h>

void PIDControlInit ( PIDControllerContext_t *ctx )
{
	ctx->err 		= 0;
	ctx->prevErr	= 0;
	ctx->integrSum 	= 0;
}


controllerRensponse_t PIDControlResponse ( PIDControllerContext_t *ctx )
{
    controllerRensponse_t control = 0;

    ctx->integrSum += ctx->err;
    /* Symmetric limits */
    ctx->integrSum = CLIP_VALUE( ctx->integrSum, -ctx->integrLimit, ctx->integrLimit );
    
    control = ctx->kp * ctx->err +
                ctx->ki * ctx->integrSum +
                ctx->kd * (ctx->err - ctx->prevErr);

    ctx->prevErr = ctx->err;

    return control;
}