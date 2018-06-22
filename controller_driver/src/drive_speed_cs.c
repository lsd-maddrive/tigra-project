#include <drive_speed_cs.h>

/* Configure minimum and maximum speed reference values (rpm)*/
#define speedReferenceMaxVal   30000
#define speedReferenceMinVal   0

typedef float         controllerRate_t;

/* Configuration - PID controller parameters*/
typedef struct {

  controllerRate_t      kp;
  controllerRate_t      ki;
  controllerRate_t      kd;

  float                 integrSum;

} controllerParams_t;

/* Switching context - input parameter to controller function */
typedef struct {

    wheelVelocity_t     err;
    controllerParams_t  *params;

} controllerContext_t;

/* Configuration - PID controller parameters*/
static controllerParams_t PIDParmsCfg = {
    .kp   = 1,
    .ki   = 0.1,
    .kd   = 0,
    .integrSum  = 0
};

/* Switching context - input parameter to controller function */
static controllerContext_t PIDContext = {
    .err    = 0,
    .params = &PIDParmsCfg
};

static bool isInitialized = false;

/*
 *@brief         Low level drivers initialization
 *@note          First time function call sets flag "isInitialized"
 *               which protects of multiple initialization
 */
void DriveSpeedCSInit( void )
{
    if ( isInitialized )
        return
    
    wheelPosSensorInit();
    lldControlInit();
    
    isInitialized = true;
}


/*
 *  @brief       Control system law realization. PID controller *
 *  @params[in]  Structure, contains current error, PID controller parameters
 *  @params[out] Controller output [0;100] %
 *  @note        Access parameters like this:
 *                                            PIDContext->params->kp
 *                                            PIDContext->err
 */
static int32_t PIDController ( controllerContext_t *ctx )
{
    int32_t control = 0;

    ctx->params->integrSum += ctx->err;
    if ( ctx->params->integrSum > 100 )
        ctx->params->integrSum = 100;
    else if ( ctx->params->integrSum < -100 )
        ctx->params->integrSum = -100;
    
    control = ctx->params->kp * ctx->err +
                ctx->params->ki * ctx->params->integrSum;

    return control;
}


/*
 * @brief        Control system "shell".
 *               Function calculates the difference between reference and
 *               current speed. Calls the controller function which calculates
 *               control action value. Sets the power value (and DIRECION?)
 *               to the motor.
 * @params [in]  Speed reference value
 * @note         If speed reference value doesn't match the limits
 *               it will be saturated
 * @return       Controller output, if all required lld's is initialized
 *               -1               , if not
 */
int32_t DriveSpeedControl ( wheelVelocity_t speedReference )
{
  /* Check if all modules initialized. if not return -1 */
    if( !isInitialized )
        return -1;

    /* Speed reference saturation */
    if ( speedReference > speedReferenceMaxVal )
        speedReference = speedReferenceMaxVal;
    else if ( speedReference < speedReferenceMinVal )
        speedReference = speedReferenceMinVal;

    wheelVelocity_t currentSpeed    =   wheelPosSensorGetVelocity ();
    PIDContext.err  =   speedReference - currentSpeed;

    int32_t lldMotorPower = PIDController(&PIDContext);

    lldControlSetDrMotorPower ( lldMotorPower );

    return lldMotorPower;
}
















