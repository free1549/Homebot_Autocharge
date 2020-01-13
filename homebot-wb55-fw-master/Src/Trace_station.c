#include "Ir_trace.h"
#include "motor.h"
#include "app_conf.h"
#include "stm32_seq.h"
#include "timer.h"
#include "trace_station.h"
#include "simple.pb-c.h"
#include "hi2st-parser.h"

typedef enum
{
    SETED_DIRECTION = 0,
    NEED_SET_DIRECTION,
    DONT_NEED_SET,
}SET_DIRECTION;

typedef enum
{
    FIND_LONG = 1,
    FIND_SHORT,
}FIND_MODE;

typedef enum
{
    DETECTED = 0,
    FORWARD,
    BACKWARD,
    GOLEFT,
    FASTGOLEFT,
    GORIGHT,
    FASTGORIGHT,
    TURNLEFT,
    TURNRIGHT,  
    STOP,
    NOTDIRECTION,
}MOTION;

typedef enum
{
    CANT_DETECT = 0,
    VERY_LOW,
    LOW,
    MIDDLE,
    HIGH,
    VERY_HIGH,
    PERFECT,
}IR_POWER;

typedef enum
{
    LEFT = 0,
    RIGHT = 1,
}DIRECTION;

typedef enum
{
    SUCCESS_DETECTED = 0,
    DO_TRACE_LONG_IR,
    FAIL_DETECTED,
}RESULT_TRACE_LONG;

typedef enum
{
    MODE_FIND_SHORT_IR = 0,    
    MODE_SET_SHORT_DIRECTION,
    MODE_GOTO_OUTLINE,
    MODE_FIND_LONG_IR,
    MODE_SET_LONG_DIRECTION,
    MODE_GOTO_CENTERLINE,
    MODE_TRACE_LONG_IR,
    //MODE_TRACE_SHORT_IR,
    MODE_COMPLETE,
}MODE_TRACE_STATION;

typedef enum
{
    CANT_FIND = 0,
    FIND_CENTER,
    FIND_LEFT,
    FIND_RIGHT,
    FIND_SHORT_CENTER,
    FIND_SHORT_LEFT,
    FIND_SHORT_RIGHT,
    REPEAT,
}RESULT_FIND_STATION;

typedef enum
{
    FIND_LEVEL1 = 0,
    FIND_LEVEL2,
    FIND_LEVEL3,
}FIND_LEVEL;

typedef struct
{
    int right;
    int center;
    int left;
}COUNT_DIRECTION;

#define SET_DIRECTION_TIME 175 //*20ms 125
#define SWING_TIME 550 //*20ms
#define MOVE_TIME 400 //*20ms
#define STOP_THRES 3
#define AVOID_TIME 100 //*20ms
#define FIND_TIME 500 //*20ms
#define NONE 0xFFFFFFFF
#define AROUNDTIME 5000 //*20ms
#define DEFAULT_SPEED 12

#define YAW_FTOI(yaw) (int)(yaw*0.1)
#define IR_COUNT(num) (num*5)
#define GET_IR_POWER_LEVEL(currentdirection) (currentdirection.long_distance.bytes.long_r +\
                                                  currentdirection.long_distance.bytes.long_c +\
                                                  currentdirection.long_distance.bytes.long_l)
#define IS_DETECTED_ALL_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r >= 1\
                                                        && currentdirection.long_distance.bytes.long_c >= 1\
                                                        && currentdirection.long_distance.bytes.long_l >= 1)
#define IS_DETECTED_LEFT_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r == 0\
                                                        && currentdirection.long_distance.bytes.long_l >= 1)
#define IS_DETECTED_LEFT_CENTER_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r == 0\
                                                        && currentdirection.long_distance.bytes.long_c >= 1\
                                                        && currentdirection.long_distance.bytes.long_l >= 1)
#define IS_DETECTED_RIGHT_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r >= 1\
                                                        && currentdirection.long_distance.bytes.long_l == 0)
#define IS_DETECTED_RIGHT_CENTER_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r >= 1\
                                                        && currentdirection.long_distance.bytes.long_c >= 1\
                                                        && currentdirection.long_distance.bytes.long_l == 0)
#define IS_DETECTED_CENTER_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r == 0\
                                                        && currentdirection.long_distance.bytes.long_c >= 1\
                                                        && currentdirection.long_distance.bytes.long_l == 0)
#define IS_DETECTED_SIDE_DIRECTION(currentdirection) (currentdirection.long_distance.bytes.long_r == 1\
                                                        && currentdirection.long_distance.bytes.long_c >= 0\
                                                        && currentdirection.long_distance.bytes.long_l == 1)
#define IS_DETECTED_ALL_SHORT_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_r >= 1\
                                                        && currentdirection.short_distance.bytes.short_c >= 1\
                                                        && currentdirection.short_distance.bytes.short_l >= 1)
#define IS_DETECTED_SHORT_DIRECTION_(currentdirection) (currentdirection.short_distance.bytes.short_r >= 1\
                                                        || currentdirection.short_distance.bytes.short_c >= 1\
                                                        || currentdirection.short_distance.bytes.short_l >= 1)
#define IS_DETECTED_SHORT_DIRECTION(currentdirection) ((currentdirection.short_distance.bytes.short_r >= 1\
                                                        && currentdirection.short_distance.bytes.short_c >= 1)\
                                                        || (currentdirection.short_distance.bytes.short_l >= 1\
                                                        &&  currentdirection.short_distance.bytes.short_c >= 1))
#define IS_DETECTED_SHORT_ALL_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_r >= 1\
                                                        && currentdirection.short_distance.bytes.short_c >= 1\
                                                        && currentdirection.short_distance.bytes.short_l >= 1)
#define IS_DETECTED_SHORT_CENTER_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_r == 0\
                                                            && currentdirection.short_distance.bytes.short_c >= 1\
                                                            && currentdirection.short_distance.bytes.short_l == 0)
#define IS_DETECTED_SHORT_ONLY_CENTER_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_c >= 1)
#define IS_DETECTED_SHORT_LEFT_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_r == 0\
                                                        && currentdirection.short_distance.bytes.short_l >= 1)
#define IS_DETECTED_SHORT_RIGHT_DIRECTION(currentdirection) (currentdirection.short_distance.bytes.short_l == 0\
                                                        && currentdirection.short_distance.bytes.short_r >= 1)
#define IS_MAX_CENTER(irdirection) (irdirection.center > irdirection.right && irdirection.center > irdirection.left)
#define IS_MAX_LEFT(irdirection) (irdirection.left > irdirection.right && irdirection.left > irdirection.center)
#define IS_MAX_RIGHT(irdirection) (irdirection.right > irdirection.center && irdirection.right > irdirection.left)

extern UART_HandleTypeDef huart1;
extern motor_TypeDef DCMOTOR;
static int TaskStep, TraceMode; 
static MODE_DEBUG debug;

static void Move_Robot(int lspeed, int rspeed)
{
    DCMOTOR.Value.l_cmdrpm = lspeed;
    DCMOTOR.Value.r_cmdrpm = rspeed;
}

static void Move_Forward(int speed)
{
    Move_Robot(speed, speed);
}

static void Move_Backward(int speed)
{
    Move_Robot(-speed, -speed);
}

static void Turn_Left()
{
    Move_Robot(-DEFAULT_SPEED+3, DEFAULT_SPEED-3);
}

static void Turn_Right()
{
    Move_Robot(DEFAULT_SPEED-3, -DEFAULT_SPEED+3);
}

static void Stop_Robot()
{
    Move_Robot(0, 0);
}

static void Move_Goaround_Left()
{
    Move_Robot(DEFAULT_SPEED, DEFAULT_SPEED-5);
}

static void Move_Goaround_Right()
{
    Move_Robot(DEFAULT_SPEED-5, DEFAULT_SPEED);
}

static void Move_Goaround_FastRight()
{
    Move_Robot(DEFAULT_SPEED-5, DEFAULT_SPEED+3); 
}

static void Move_Goaround_FastLeft()
{
    Move_Robot(DEFAULT_SPEED+3, DEFAULT_SPEED-5);
}


static void Move_Swing_Left(int lspeed, int rspeed)
{
    Move_Robot(lspeed, rspeed);
}

static void Move_Swing_Right(int lspeed, int rspeed)
{
    Move_Robot(lspeed, rspeed);
}
static int Select_By_Long_Direction(IR_DIRECTION* direction, int reset)
{
    static int CenterShortCount, LeftShortCount, RightShortCount;
    if(reset == 1)
    {
        CenterShortCount = LeftShortCount = RightShortCount = 0;
        return 255;
    }
    
    IR_DIRECTION currentdirection = *direction;

    #if 0
    if(IS_DETECTED_SHORT_CENTER_DIRECTION(currentdirection))
    {        
        CenterShortCount++;
        if(CenterShortCount >= IR_COUNT(2))
        {
            CenterShortCount = 0;
            return DETECTED;
        }
    }
    else if(IS_DETECTED_SHORT_LEFT_DIRECTION(currentdirection))
    {
        LeftShortCount++;
        if(LeftShortCount >= IR_COUNT(2))
        {
            LeftShortCount = 0;
            return DETECTED;
        }
    }
    else if(IS_DETECTED_SHORT_RIGHT_DIRECTION(currentdirection))
    {
        RightShortCount++;
        if(RightShortCount >= IR_COUNT(2))
        {
            RightShortCount = 0;
            return DETECTED;
        }
    }
    #endif
    #if 0
    if(IS_DETECTED_SHORT_ONLY_CENTER_DIRECTION(currentdirection))
    {        
        return FORWARD;
        CenterShortCount++;
        if(CenterShortCount >= IR_COUNT(2))
        {
            CenterShortCount = 0;
            return DETECTED;
        }
    }
    #endif
    //if(IS_DETECTED_ALL_DIRECTION(currentdirection) || IS_DETECTED_CENTER_DIRECTION(currentdirection) || IS_DETECTED_SIDE_DIRECTION(currentdirection))
    if(IS_DETECTED_ALL_DIRECTION(currentdirection) || IS_DETECTED_CENTER_DIRECTION(currentdirection))
    {     
        //CenterShortCount = LeftShortCount = RightShortCount = 0;
        return FORWARD;
    }
    #if 0
    else if(IS_DETECTED_LEFT_CENTER_DIRECTION(currentdirection))
    {
        CenterShortCount = LeftShortCount = RightShortCount = 0;
        return FASTGOLEFT;
    }
    else if(IS_DETECTED_RIGHT_CENTER_DIRECTION(currentdirection))
    {
        CenterShortCount = LeftShortCount = RightShortCount = 0;
        return FASTGORIGHT;
    }
    #endif
    #if 0
    else if(IS_DETECTED_SIDE_DIRECTION(currentdirection))
    {
        return NOTDIRECTION;
    }
    #endif
    else if(IS_DETECTED_RIGHT_DIRECTION(currentdirection))
    {
        //CenterShortCount = LeftShortCount = RightShortCount = 0;
        return GOLEFT;
    }
    else if(IS_DETECTED_LEFT_DIRECTION(currentdirection))
    {
        //CenterShortCount = LeftShortCount = RightShortCount = 0;
        return GORIGHT;        
    }
    else
    {
        //CenterShortCount = LeftShortCount = RightShortCount = 0;
        return STOP;
    }
    return STOP;
}

static void Action_Robot(int motion, int reset)
{
    if(reset == 1)
    {
        return;
    }
    switch(motion)
    {
        case STOP :
        {
            Stop_Robot();
            break;
        }
        case FORWARD:
        {
            Move_Forward(DEFAULT_SPEED);
            break;
        }
        case BACKWARD:
        {
            Move_Backward(DEFAULT_SPEED);
            break;
        }
        case GOLEFT :
        {
            Move_Goaround_Left();
            break;
        }
        case GORIGHT :
        {
            Move_Goaround_Right();
            break;
        }
        case TURNLEFT :
        {
            Turn_Left();
            break;
        }
        case TURNRIGHT :
        {
            Turn_Right();
            break;
        }
        case FASTGOLEFT :
        {
            Move_Goaround_FastLeft();
            break;
        }
        case FASTGORIGHT :
        {
            Move_Goaround_FastRight();
            break;
        }
        case DETECTED :
        {
            Stop_Robot();
            break;
        }
        default :
        {
            break;
        }
    }
}

static int Action_TraceLongIr(IR_DIRECTION* direction, int* currentmotion, int reset)
{
    static int StopCount, OutlineTime, MissingDirection;
    static uint8_t FlagMissForward;
    static int SaveMotion = FORWARD;
    if(reset == 1)
    {
        StopCount = OutlineTime = MissingDirection = FlagMissForward = 0;
        SaveMotion = FORWARD;
        return 255;
    }
    
    if(StopCount >= STOP_THRES)
    {
        MissingDirection++;
        if(MissingDirection >= 4)
        {
            #if 0
            Stop_Robot();
            #endif
            StopCount = 0;
            SaveMotion = FORWARD;
            OutlineTime = 0;
            MissingDirection = 0;
            FlagMissForward = 0;

            return FAIL_DETECTED;
        }
        else if(MissingDirection == 2)
        {
            FlagMissForward++;
        }
		StopCount = 0;
        OutlineTime = AVOID_TIME;
    }

    if(OutlineTime)
    {
        if(SaveMotion == GOLEFT)
        {
            *currentmotion = GORIGHT;
        }
        else if(SaveMotion == GORIGHT)
        {
            *currentmotion = GOLEFT;
        }
        else if(SaveMotion == FORWARD)
        {
            if(!FlagMissForward)
            {
                *currentmotion = TURNLEFT;
            }
            else
            {
                *currentmotion = TURNRIGHT;
            }
        }
        else
        {

        }
        OutlineTime--;
    }
    else
    {
        *currentmotion = Select_By_Long_Direction(direction, 0);
        
        if(*currentmotion == GOLEFT || *currentmotion == GORIGHT || *currentmotion == FORWARD)
        {
            MissingDirection = 0;
            FlagMissForward = 0;
			StopCount = 0;
            SaveMotion = *currentmotion;
        }
        #if 0
        else if(*currentmotion == NOTDIRECTION)
        {
            SaveMotion == FORWARD;
            StopCount = STOP_THRES;;
        }
        #endif
        else if(*currentmotion == STOP)
        {
            StopCount++;
        }
        else if(*currentmotion == DETECTED)
        {
            return SUCCESS_DETECTED;
        }
    }
    return DO_TRACE_LONG_IR;
}

static int Action_FindStation(IR_DIRECTION* direction, int mode, int reset)
{
    static int FindLevel, FindResult;
    static int FindCountLongCenter, FindCountLongLeft, FindCountLongRight, FindCountShortCenter, FindCountShortLeft, FindCountShortRight;
    static int FindTime = FIND_TIME;
    if(reset == 1)
    {
        FindLevel = FindResult = FindCountLongCenter = FindCountLongLeft = FindCountLongRight
        = FindCountShortCenter = FindCountShortLeft = FindCountShortRight = 0;
        FindTime = FIND_TIME;
        return 255;
    }
    
    IR_DIRECTION currentdirection = *direction;
    int motion = 0;
    debug.detectirstep = FindLevel;
    
    if(FindTime > 0)
    {
        if(FindLevel == FIND_LEVEL1)
        {
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_ALL_DIRECTION(currentdirection))
                {
                    FindCountLongCenter++;
                    if(FindCountLongCenter >= IR_COUNT(2))
                    {
                        FindResult = FIND_CENTER;
                    }
                }
                else
                {
                    FindCountLongCenter = 0;
                    motion = TURNLEFT;
                }
            }
            if(mode == FIND_SHORT)
            {
                if(IS_DETECTED_SHORT_ALL_DIRECTION(currentdirection))
                {
                    FindCountShortCenter++;   
                    if(FindCountShortCenter >= IR_COUNT(2))
                    {
                        FindResult = FIND_SHORT_CENTER;
                    }
                }
                else
                {                    
                    FindCountShortCenter = 0;
                    motion = TURNLEFT;
                }
            }            
        }
        else if(FindLevel == FIND_LEVEL2)
        {            
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_LEFT_DIRECTION(currentdirection))
                {
                    FindCountLongLeft++;
                    if(FindCountLongLeft >= IR_COUNT(2))
                    {
                        FindResult = FIND_LEFT;
                    }
                }
                else
                {
                    FindCountLongLeft = 0;
                    motion = TURNLEFT;
                }
            }
            if(mode == FIND_SHORT)
            {
                if(IS_DETECTED_SHORT_LEFT_DIRECTION(currentdirection))
                {       
                    FindCountShortLeft++;
                    if(FindCountShortLeft >= IR_COUNT(2))
                    {
                        FindResult = FIND_SHORT_LEFT;
                    }
                }
                else
                {                    
                    FindCountShortLeft = 0;
                    motion = TURNLEFT;
                }
            }
        }
        else if(FindLevel == FIND_LEVEL3)
        {      
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_RIGHT_DIRECTION(currentdirection))
                {   
                    FindCountLongRight++;
                    if(FindCountLongRight >= IR_COUNT(2))
                    {
                        FindResult = FIND_RIGHT;
                    }
                    
                }
                else
                {
                    FindCountLongRight = 0;
                    motion = TURNLEFT;
                }   
            }
            if(mode == FIND_SHORT)
            {                
                if(IS_DETECTED_SHORT_RIGHT_DIRECTION(currentdirection))
                {     
                    FindCountShortRight++;
                    if(FindCountShortRight >= IR_COUNT(2))
                    {
                        FindResult = FIND_SHORT_RIGHT;
                    }
                }
                else
                {                   
                    FindCountShortRight = 0;
                    motion = TURNLEFT;
                }
            }
        }

        if(FindResult)
        {            
            int returnvalue = 0;
            returnvalue = FindResult;
            
            FindTime = FIND_TIME;
            motion = STOP;
            FindLevel = 0;
            FindResult = 0;
            FindCountLongCenter = FindCountLongLeft = FindCountLongRight
            = FindCountShortCenter = FindCountShortLeft = FindCountShortRight = 0;
            
            Action_Robot(motion, 0);
            
            return returnvalue; 
        }
        
        Action_Robot(motion, 0);
        FindTime--;
    }
    else
    {
        FindTime = FIND_TIME;
        motion = STOP;
        Action_Robot(motion, 0);
        FindLevel++;
        
        if(FindLevel > FIND_LEVEL3)
        {
            FindLevel = 0;
            return CANT_FIND;
        }
    }

    return REPEAT;
}

static int Action_SetStartDirection(int* startdirection, int reset)
{
    static int CountSetDirection = SET_DIRECTION_TIME;
    if(reset == 1)
    {
        CountSetDirection = SET_DIRECTION_TIME;
        return 255;
    }
    
    if(CountSetDirection > 0)
    {
        if(*startdirection == FIND_SHORT_LEFT || *startdirection == FIND_LEFT)
        {
            Turn_Left();
        }
        else if(*startdirection == FIND_SHORT_RIGHT || *startdirection == FIND_RIGHT)
        {
            Turn_Right();
        }
        else if(*startdirection == FIND_SHORT_CENTER || *startdirection == FIND_CENTER)
        {
            CountSetDirection = SET_DIRECTION_TIME;
            return DONT_NEED_SET; 
        }
        if(*startdirection == FIND_SHORT_LEFT || *startdirection == FIND_SHORT_RIGHT || *startdirection == FIND_SHORT_CENTER)
        {
            CountSetDirection--;
        }
        else
        {
            CountSetDirection--;
        }
    }
    else
    {
        Stop_Robot();
        CountSetDirection = SET_DIRECTION_TIME;
        return SETED_DIRECTION;
    }

    return NEED_SET_DIRECTION;
}

static int Action_TracePhase0(IR_DIRECTION* direction, int* startdirection, int reset)
{
    int result = 0;
    if(reset == 1)
    {
        return 255;
    }

    result = Action_FindStation(direction, FIND_SHORT, 0);
    *startdirection = debug.detectlev = result;

    if(result == FIND_SHORT_CENTER || result == FIND_SHORT_LEFT || result == FIND_SHORT_RIGHT)
    {
        return NEED_SET_DIRECTION;
    }
    else if(result == REPEAT)
    {        
        return MODE_FIND_SHORT_IR;
    }    
    else if(result == CANT_FIND)
    {
        return MODE_FIND_LONG_IR;
        //error
    }
    else
    {

    }
}

static int Action_TracePhase0point5(IR_DIRECTION* direction, int* startdirection, int reset)
{
    int result = 0;
    if(reset == 1)
    {
        return 255;
    }

    result = Action_SetStartDirection(startdirection, 0);
    
    if(result == DONT_NEED_SET)
    {
        return MODE_TRACE_LONG_IR;
    }
    else if(result == SETED_DIRECTION)
    {
        return MODE_GOTO_OUTLINE;
    }
    else if(result == NEED_SET_DIRECTION)
    {
        return MODE_SET_SHORT_DIRECTION;
    }
    else
    {
        return MODE_SET_SHORT_DIRECTION;
    }    
}

static int Action_TracePhase0point8(int reset)
{    
    static int MoveCount = MOVE_TIME;
    if(reset == 1)
    {
        MoveCount = MOVE_TIME;
        return 255;
    }

    if(MoveCount > 0)
    {
        Move_Forward(DEFAULT_SPEED);
        MoveCount--;
    }
    else
    {
        MoveCount = MOVE_TIME;
        return MODE_FIND_LONG_IR;
    }
	
	return MODE_GOTO_OUTLINE;
}


static int Action_TracePhase1(IR_DIRECTION* direction, int* startdirection, int reset)
{
    int result = 0;
    if(reset == 1)
    {
        return 255;
    }

    result = Action_FindStation(direction, FIND_LONG, 0);
    *startdirection = debug.detectlev = result;

    if(result == FIND_CENTER)
    {
        return MODE_TRACE_LONG_IR;
    }
    else if(result == FIND_LEFT || result == FIND_RIGHT)
    {
        return MODE_SET_LONG_DIRECTION;
        return MODE_TRACE_LONG_IR;
    }
    else if(result == REPEAT)
    {        
        return MODE_FIND_LONG_IR;
    }    
    else if(result == CANT_FIND)
    {
        return MODE_FIND_SHORT_IR;
        //error
    }
    else
    {

    }
}

static int Action_TracePhase1point5(IR_DIRECTION* direction, int* startdirection, int reset)
{
    int result = 0;
    if(reset == 1)
    {
        return 255;
    }

    result = Action_SetStartDirection(startdirection, 0);
    
    if(result == DONT_NEED_SET)
    {
        return MODE_TRACE_LONG_IR;
    }
    else if(result == SETED_DIRECTION)
    {
        return MODE_GOTO_CENTERLINE;
    }
    else if(result == NEED_SET_DIRECTION)
    {
        return MODE_SET_LONG_DIRECTION;
    }
    else
    {
        return MODE_SET_LONG_DIRECTION;
    }    
}

static int Action_TracePhase1point8(int reset)
{    
    static int MoveCount = MOVE_TIME*2;
    if(reset == 1)
    {
        MoveCount = MOVE_TIME*2;
        return 255;
    }

    if(MoveCount > 0)
    {
        Move_Forward(DEFAULT_SPEED);
        MoveCount--;
    }
    else
    {
        MoveCount = MOVE_TIME*2;
        return MODE_FIND_LONG_IR;
    }
	
	return MODE_GOTO_CENTERLINE;
}


static int Action_TracePhase2(IR_DIRECTION* direction, int* startdirection, int reset)
{
    int currentmotion, result = 0;
    if(reset == 1)
    {
        return 255;
    }
    
    result = Action_TraceLongIr(direction, &currentmotion, 0);
    Action_Robot(currentmotion, 0);
    debug.direction = currentmotion;

    //Check On Station : mode complete

    if(result == FAIL_DETECTED)
    {
        return MODE_FIND_SHORT_IR;
    }
    
    return MODE_TRACE_LONG_IR;
}

static int Action_TracePhase3(IR_DIRECTION* direction, int reset)
{
    static int StopCount = 200;
    if(reset == 1)
    {
        StopCount = 200;
        return 255;
    }

    if(StopCount > 0)
    {
        Move_Forward(DEFAULT_SPEED);
        StopCount--;
    }
    else
    {
        #if 1
        ToHi tohi = TO_HI__INIT;
        tohi.has_do_charge = 1;
        tohi.do_charge = 1;
        pushToHi(&huart1, &tohi);
        #endif
        StopCount = 200;
        return MODE_FIND_SHORT_IR;
    }
	
	return MODE_COMPLETE;
}

void Charge_Process()
{   
    #if 1
    static int TraceStationMode = MODE_FIND_SHORT_IR;
    static int StartDirection;
    int chargepg = 0;
    if(Get_ChargeMode() != 1)
    {
        TraceStationMode = MODE_FIND_SHORT_IR;
        StartDirection = 0;
        return;
    }

    #if 0
    ToHi tohi = TO_HI__INIT;          
    tohi.has_do_charge = 1;
    tohi.do_charge = 1;
    pushToHi(&huart1, &tohi);
    #endif
    
    IR_DIRECTION current_ir, before_ir;    
    Ir_Getdata(&current_ir, &before_ir);
    
    chargepg = ST_READ_PG;    
    if(!chargepg)
    {
        TraceStationMode = MODE_COMPLETE;
    }
    
    debug.ir.long_distance.ir_long = current_ir.long_distance.ir_long;
    debug.ir.short_distance.ir_short = current_ir.short_distance.ir_short;
    debug.processstep = TraceStationMode;
    debug.powergood = chargepg;        
    
    #if 1
    switch(TraceStationMode)
    {
        case MODE_FIND_SHORT_IR :
        {
            TraceStationMode = Action_TracePhase0(&current_ir, &StartDirection, 0);
            break;
        }
        case MODE_SET_SHORT_DIRECTION :
        {
            TraceStationMode = Action_TracePhase0point5(&current_ir, &StartDirection, 0);
            break;
        }
        case MODE_GOTO_OUTLINE :
        {
            TraceStationMode = Action_TracePhase0point8(0);
            break;
        }
        case MODE_FIND_LONG_IR :
        {            
            TraceStationMode = Action_TracePhase1(&current_ir, &StartDirection, 0);
            break;
        }
        case MODE_SET_LONG_DIRECTION :
        {
            TraceStationMode = Action_TracePhase1point5(&current_ir, &StartDirection, 0);
            break;
        }
        case MODE_GOTO_CENTERLINE :
        {
            TraceStationMode = Action_TracePhase1point8(0);
            break;
        }
        case MODE_TRACE_LONG_IR :
        {
            TraceStationMode = Action_TracePhase2(&current_ir, &StartDirection, 0);
            break;
        }
        case MODE_COMPLETE :
        {
            TraceStationMode = Action_TracePhase3(&current_ir, 0);
            break;
        }
        default :
        {
            break;
        }
    }
    #endif
    
    #if 0
    static int FCount = 250;
    static int RCount;

    if(RCount)
    {
        Move_Forward(90);
    }
    else
    {
        Stop_Robot();
    }


    if(FCount > 0)
    {
        FCount--;
    }
    else
    {
        FCount = 250;
        RCount ^= 1;
    }
    #endif
    
#ifdef MOTOR_TEST
    static int TestCount = 200;
    static int TestFlag = 0;
    static int TestValue;
    if(TestCount > 0)
    {
        TestCount--;
    }
    else
    {
        TestCount = 200;
        TestFlag ^= 1;
    }

    if(TestFlag == 1)
    {
        Move_Forward(TestValue);
    }
    else
    {
        Move_Backward(TestValue);
    }
#endif
#endif
}

void Clear_InnerStaticVar()
{
    Action_TraceLongIr(0, 0, 1);
    Action_FindStation(0, 0, 1);
    Action_SetStartDirection(0, 1);
    Action_TracePhase0point8(1);
    Action_TracePhase3(0, 1);
    Charge_Process();
    Stop_Robot();
}

int Get_ChargeMode()
{
    return TraceMode;
}

void Set_ChargeMode()
{
    TraceMode = 1;   
    
}

void Clear_ChargeMode()
{
    TraceMode = 0;
}

MODE_DEBUG Get_Debug()
{
    return debug;
}

#if 0
static int Action_TraceShortIr(IR_DIRECTION* direction, int* currentmotion, int reset)
{
    static int StopCount, SaveMotion, OutlineTime, MissingDirection;
    static uint8_t FlagMissForward;
    if(reset == 1)
    {
        StopCount = SaveMotion = OutlineTime = MissingDirection = FlagMissForward = 0;
        return 255;
    }
    
    if(StopCount >= STOP_THRES)
    {
        MissingDirection++;
        if(MissingDirection >= 4)
        {
            #if 0
            Stop_Robot();
            #endif
            StopCount = 0;
            SaveMotion = FORWARD;
            OutlineTime = 0;
            MissingDirection = 0;
            FlagMissForward = 0;

            return FAIL_DETECTED;
        }
        else if(MissingDirection == 2)
        {
            FlagMissForward++;
        }
		StopCount = 0;
        OutlineTime = AVOID_TIME;
    }

    if(OutlineTime)
    {
        if(SaveMotion == GOLEFT)
        {
            *currentmotion = GORIGHT;
        }
        else if(SaveMotion == GORIGHT)
        {
            *currentmotion = GOLEFT;
        }
        else if(SaveMotion == FORWARD)
        {
            if(!FlagMissForward)
            {
                *currentmotion = TURNLEFT;
            }
            else
            {
                *currentmotion = TURNRIGHT;
            }
        }
        else
        {

        }
        OutlineTime--;
    }
    else
    {
        *currentmotion = Choice_Swing_Direction(direction);
        
        if(*currentmotion == GOLEFT || *currentmotion == GORIGHT || *currentmotion == FORWARD)
        {
            MissingDirection = 0;
            FlagMissForward = 0;
			StopCount = 0;
            SaveMotion = *currentmotion;
        }
        else if(*currentmotion == STOP)
        {
            StopCount++;
        }
        else if(*currentmotion == DETECTED)
        {
            return SUCCESS_DETECTED;
        }
    }
    
    return DO_TRACE_LONG_IR;
}
#endif

#if 0
static void Count_Direction(IR_DIRECTION* currentdirection, COUNT_DIRECTION* countdirection)
{
    IR_DIRECTION CurrentDirection = *currentdirection;
    
    if(IS_DETECTED_ALL_DIRECTION(CurrentDirection))
    {
        countdirection->center++;
    }
    else if(IS_DETECTED_LEFT_DIRECTION(CurrentDirection))
    {
        countdirection->left++;
    }
    else if(IS_DETECTED_RIGHT_DIRECTION(CurrentDirection))
    {
        countdirection->right++;
    }
    else if(IS_DETECTED_CENTER_DIRECTION(CurrentDirection))
    {
        countdirection->center++;
    }
    else
    {     
        
    }
}

#define GAP 2.0f

static int Is_Same_Position(float* currentpoint, float* startpoint)
{
    float sum = 0;
    float minus = 0;
    float min = 0;
    float max = 0;

    minus = *startpoint - GAP;
    if(minus < 0)
    {
       min = minus + 360.0f; 
    }
    else
    {
       min = minus;
    }
    
    sum = *startpoint + GAP;
    if(sum > 360.0f)
    {
        max = sum - 360.0f;
    }
    else
    {
        max = sum;
    }

    if(*currentpoint >= min && *currentpoint <= max)
    {
        return 1;
    }
    else
    {
        if(sum > 360.0f)
        {
            if((*currentpoint >= min && *currentpoint < 360.0f) || (*currentpoint <= max && *currentpoint >= 0.0f))
            {
                return 1;
            }
        }

        if(minus < 0)
        {
            if((*currentpoint <= max && *currentpoint > 0.0f) || (*currentpoint >= min && *currentpoint < 360.0f))
            {
                return 1;
            }
        }
    }
    
    return 0;
    
}

__weak static void Action_DetectIr1(int* irpoint, int* irlevel, COUNT_DIRECTION* irdirection)
{
    IR_DIRECTION currentdirection, beforedirection;
    SYSTEM_key sysdata;
    Ir_Getdata(&currentdirection, &beforedirection);
    Sys_Getdata(&sysdata);

    static int turncount, nfirst, delay;
    static float startpoint, destcount[6], destpoint[6];
    float currentpoint = 0;
    static COUNT_DIRECTION destdirection;
    

    if(turncount < 3)//if((currentpoint != startpoint) && (turncount < 3))
    {
        if(!nfirst)
        {            
            //startpoint = YAW_FTOI(sysdata.yaw);
            startpoint = sysdata.yaw;
            nfirst = TRUE;                    
            Turn_Left();
            setTimer(Timer_CUSTOM, 3000, TIME_RESET);
            #ifdef TIMER
            setTimer(Timer_CUSTOM, 1000, TIME_RESET);
            #endif
        }
        else
        {
            if(onCheckTimer(Timer_CUSTOM))
            {
                delay = 1;
            }

            if(delay)
            {
                Turn_Left();
                //currentpoint = YAW_FTOI(sysdata.yaw);
                currentpoint = sysdata.yaw;

                #ifdef TIMER
                if(onCheckTimer(Timer_CUSTOM))
                {
                    turncount = 3;
                }
                #else
                if(Is_Same_Position(&currentpoint, &startpoint))
                {
                    turncount++;
                }
                #endif

                uint8_t irpowerlevel = GET_IR_POWER_LEVEL(currentdirection);

                destpoint[irpowerlevel] += currentpoint;
                destcount[irpowerlevel]++;
                Count_Direction(&currentdirection, &destdirection);
            }
        }
    }
    else
    {
        Stop_Robot();
        for(uint8_t i=(sizeof(destpoint)/4)-1; i>0; i--)
        {
            if(destpoint[i])
            {
                destpoint[i] /= destcount[i];
                *irpoint = destpoint[i];
                *irlevel = i;
                *irdirection = destdirection;
                
                break;
            }
            else
            {
                *irpoint = CANT_DETECT;
                *irlevel = CANT_DETECT;
                irdirection->center = 0;
                irdirection->left = 0;
                irdirection->right = 0;
            }
        }
        startpoint = turncount = nfirst = delay = 0;
        for(uint8_t i=0; i<(sizeof(destpoint)/4)-1; i++)
        {
            destpoint[i] = destcount[i] = 0;
        }
        destdirection.center = destdirection.left = destdirection.right = 0;
        
        TaskStep++;
    }
}

static void Detect_Direction(uint8_t* deepsearch, uint8_t* cirpowerlevel, IR_DIRECTION direction, COUNT_DIRECTION* destdirection)
{
    IR_DIRECTION currentdirection = direction;
    
    if(*deepsearch == 0)
    {
        *cirpowerlevel = IS_DETECTED_ALL_DIRECTION(currentdirection);
        if(*cirpowerlevel)
        {
            destdirection->center++;
        }
        else
        {
            destdirection->center = 0;
        }
    }
    else if(*deepsearch == 1)
    {
        *cirpowerlevel = IS_DETECTED_LEFT_DIRECTION(currentdirection);
        if(*cirpowerlevel)
        {                        
            destdirection->left++;
        }
        else
        {
            destdirection->left = 0;
        }
    }
    else if(*deepsearch == 2)
    {
        *cirpowerlevel = IS_DETECTED_RIGHT_DIRECTION(currentdirection);
        if(*cirpowerlevel)
        {   
            destdirection->right++;
        }
        else
        {
            destdirection->right = 0;
        }
    }
    else
    {

    }
}

static void Action_DetectIr(int* irpoint, int* irlevel, COUNT_DIRECTION* irdirection)
{
    static int step;
    
    switch(step)
    {            
        case 0:
        {
            debug.detectirstep = step;
            step++;
            Turn_Left();
            setTimer(Timer_CUSTOM, 3000, TIME_RESET);
            break;
        }
        case 1:
        {
            debug.detectirstep = step;
            if(onCheckTimer(Timer_CUSTOM))
            {                
                debug.detectlev = 0;
            	setTimer(Timer_CUSTOM, 20000, TIME_RESET);
                step++;
            }
            break;
        }
        case 2:
        {
            debug.detectirstep = step;
            static uint8_t cIrPowerLevel, bIrPowerLevel, CountAcuracy, deepsearch;
            static COUNT_DIRECTION destdirection;            
            
            if(onCheckTimer(Timer_CUSTOM))
            {   
                if(deepsearch == 0)
                {
                    debug.detectlev = 1;
                    setTimer(Timer_CUSTOM, 20000, TIME_RESET);
                    deepsearch = 1;
                }
                else if(deepsearch == 1)
                {
                    debug.detectlev = 2;
                    setTimer(Timer_CUSTOM, 20000, TIME_RESET);
                    deepsearch = 2;
                }
                else
                {            
                    *irlevel = 1;                   
                    *irdirection = destdirection;
                    cIrPowerLevel = bIrPowerLevel = CountAcuracy = deepsearch = 0;
                    step++;
                }
            }
            
            Turn_Left();            

            if(Is_IrTimer())
            {            
                IR_DIRECTION currentdirection, beforedirection;
                Ir_Getdata(&currentdirection, &beforedirection);
                
                Clear_IrFlag();
                Detect_Direction(&deepsearch, &cIrPowerLevel, currentdirection, &destdirection);

                if(cIrPowerLevel == 1 && bIrPowerLevel == 1)
                {
                    CountAcuracy++;
                }
                else
                {
                    CountAcuracy = 0;
                }
                
                bIrPowerLevel = cIrPowerLevel;
                
                if(CountAcuracy >= 1)
                {
                    if(!deepsearch)
                    {
                        *irlevel = 0;
                    }
                    else
                    {
                        *irlevel = 1;
                    }
                    *irdirection = destdirection;
                    cIrPowerLevel = bIrPowerLevel = CountAcuracy = deepsearch = 0;
                    debug.detectlev = debug.detectirstep = 99;
                    step++;
                }
            }
            break;
        }   
        case 3:
        {
            debug.detectirstep = step;
            Stop_Robot();            

            static int first1 = 0;
            
            if(first1 == 0)
            {
                first1++;
                setTimer(Timer_CUSTOM, 4000, TIME_RESET);
            }
            else if(first1 == 1)
            {                
                Move_Forward(20);
            }
            else if(first1 == 2)
            {
                Move_Backward(20);
            }
            
            if(onCheckTimer(Timer_CUSTOM))
            {            
                if(first1 == 1)
                {
                    first1++;
                    setTimer(Timer_CUSTOM, 4000, TIME_RESET);
                }
                else
                {   
                    first1 = 0;
                    step = 0;
                    TaskStep++;
                }        
            }        

            break;
        }
        default :
        {
            break;
        }
    }
}

static void Action_SetDirection(int* irpoint, int* actiontask)
{
    SYSTEM_key sysdata;
    Sys_Getdata(&sysdata);
    int currentpoint = 0;
    
    currentpoint = YAW_FTOI(sysdata.yaw);

    if(currentpoint != *irpoint)
    {
        Turn_Right();
    }
    else
    {
        Stop_Robot();
        *actiontask = 1;
    }
}

static void Action_GotoStation(int* irpoint, int* irlevel)
{
    static int actiontask;
    
    if(actiontask == 0)
    {
        Action_SetDirection(irpoint, &actiontask);
    }
    else if(actiontask == 1)
    {
        Action_GoStraight(&actiontask);
    }
    else
    {
        actiontask = 0;
        *irlevel = NONE;
    }
}

static void Action_GotoStation_(int* irpoint, int* irlevel)
{
    static int actiontask = 0;
    
    if(actiontask == 0)
    {
        Action_GoStraight(&actiontask);
    }
    else if(actiontask == 1)
    {
        actiontask = 0;
        *irlevel = NONE;
    }
}


static void Action_GoAround(int direction, int* irlevel)
{
    static int nfirst;
    if(!nfirst)
    {
		nfirst = TRUE;
        setTimer(Timer_CUSTOM, AROUNDTIME, TIME_RESET);
    }
    else
    {        
        if(direction == LEFT)
        {
            Move_Goaround_Left();
        }
        else if(direction == RIGHT)
        {
            Move_Goaround_Right();
        }
        else
        {

        }
        
        if(onCheckTimer(Timer_CUSTOM))
        {
            Stop_Robot();
            nfirst = FALSE;
            *irlevel = NONE;
        }
    }
}

static void Action_ChoiceDirection(int* irpoint, int* irlevel, COUNT_DIRECTION* irdirection)
{
    COUNT_DIRECTION irdir = *irdirection;
    if(IS_MAX_CENTER(irdir))
    {
        debug.goaround = 0;
        Action_GotoStation(irpoint, irlevel);
    }
    else if(IS_MAX_LEFT(irdir))
    {
        debug.goaround = 2;
        Action_GoAround(LEFT, irlevel);
    }
    else if(IS_MAX_RIGHT(irdir))
    {
        debug.goaround = 1;
        Action_GoAround(RIGHT, irlevel);
    }
    else
    {
        debug.goaround = 3;
        Action_GoAround(LEFT, irlevel);
    }
}

static void Action_Think1(int* irpoint, int* irlevel, COUNT_DIRECTION* irdirection)
{
    switch(*irlevel)
    {
        case PERFECT:
        {
            Action_GotoStation(irpoint, irlevel);            
            break;
        }
        case VERY_HIGH:
        {
            Action_GotoStation(irpoint, irlevel);
            break;
        }
        case HIGH:
        {
            Action_ChoiceDirection(irpoint, irlevel, irdirection);
            break;
        }
        case MIDDLE:
        {
            Action_ChoiceDirection(irpoint, irlevel, irdirection);
            break;
        }
        case LOW:
        {
            Action_ChoiceDirection(irpoint, irlevel, irdirection);
            break;
        }
        case VERY_LOW:
        {
            Action_ChoiceDirection(irpoint, irlevel, irdirection);
            break;
        }
        case CANT_DETECT:
        {
            Action_GoAround(RIGHT, irlevel);
            break;
        }
        default:
        {
            TaskStep++;
            break;
        }
    }
}

static void Action_Think(int* irpoint, int* irlevel, COUNT_DIRECTION* irdirection)
{
    switch(*irlevel)
    {
        case 0:
        {
            debug.thinkstep = *irlevel;
            Action_GotoStation(irpoint, irlevel);            
            break;
        }
        case 1:
        {
            debug.thinkstep = *irlevel;
            Action_ChoiceDirection(irpoint, irlevel, irdirection);
            break;
        }
        default:
        {
            debug.thinkstep = *irlevel;
            TaskStep++;
            break;
        }
    }
}


void Trace_Station_Process_()
{   
    #if 0
    IR_DIRECTION currentdirection, beforedirection;
    Ir_Getdata(&currentdirection, &beforedirection);

    if(IS_DETECTED_ALL_SHORT_DIRECTION(currentdirection))
    {
        debug.direction = 0;
        Move_Forward(20);        
    }
    else if(IS_DETECTED_RIGHT_DIRECTION(currentdirection))
    {
        debug.direction = 1;
        Turn_Right();
    }
    else if(IS_DETECTED_ALL_DIRECTION(currentdirection) || IS_DETECTED_LEFT_DIRECTION(currentdirection)\
        || IS_DETECTED_CENTER_DIRECTION(currentdirection))
    {
        debug.direction = 2;
        Turn_Left();
    }
    else
    {
        Stop_Robot();
    }
    #endif
    #if 1
    static int irpoint, irlevel;
    static COUNT_DIRECTION irdirection;
    IR_DIRECTION c_ir, b_ir;
    Ir_Getdata(&c_ir, &b_ir);
    debug.ir.long_distance.ir_long = c_ir.long_distance.ir_long;
    debug.ir.short_distance.ir_short = c_ir.short_distance.ir_short;

    #if 0
    switch(TaskStep)
    {
        case 0:
        {
            debug.processstep = TaskStep;                
            Action_DetectIr(&irpoint, &irlevel, &irdirection);
            break;
        }
        case 1:
        {
            debug.processstep = TaskStep;
            Action_Think(&irpoint, &irlevel, &irdirection);
            break;
        }
        default :
        {
            debug.processstep = TaskStep;
            debug.thinkstep = debug.direction = debug.goaround = 99;
            TaskStep = irpoint = irpoint = 0;
            irdirection.right = irdirection.center = irdirection.left = 0;
            break;
        }
    }
    #endif
    #endif
}
#endif

#if 0
static int Action_FindStation_(IR_DIRECTION* direction, int mode)
{
    static int FindLevel, FindCount;
    static int FindTime = FIND_TIME;
    IR_DIRECTION currentdirection = *direction;
    int motion = 0;
    debug.detectirstep = FindLevel;
    
    if(FindTime > 0)
    {
        if(FindLevel == FIND_LEVEL1)
        {
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_ALL_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }
            else if(mode == FIND_SHORT)
            {
                if(IS_DETECTED_SHORT_ALL_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }            
        }
        else if(FindLevel == FIND_LEVEL2)
        {
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_LEFT_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }
            else if(mode == FIND_SHORT)
            {
                if(IS_DETECTED_SHORT_LEFT_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }
        }
        else if(FindLevel == FIND_LEVEL3)
        {
            if(mode == FIND_LONG)
            {
                if(IS_DETECTED_RIGHT_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }
            else if(mode == FIND_SHORT)
            {
                if(IS_DETECTED_SHORT_RIGHT_DIRECTION(currentdirection))
                {
                    FindCount++;        
                }
                else
                {
                    FindCount = 0;
                    motion = TURNLEFT;
                }
            }
        }

        if(FindCount >= IR_COUNT(2))
        {            
            FindTime = FIND_TIME;
            FindCount = 0;
            motion = STOP;
            Action_Robot(motion);
            if(FindLevel == FIND_LEVEL1)
            {
                return FIND_CENTER;
            }
            else if(FindLevel == FIND_LEVEL2)
            {
                FindLevel = 0;
                return FIND_LEFT;
            }
            else if(FindLevel == FIND_LEVEL3)
            {
                FindLevel = 0;
                return FIND_RIGHT;
            }
        }
        
        Action_Robot(motion);
        FindTime--;
    }
    else
    {
        FindTime = FIND_TIME;
        FindCount = 0;
        motion = STOP;
        Action_Robot(motion);
        FindLevel++;
        
        if(FindLevel > FIND_LEVEL3)
        {
            FindLevel = 0;
            return CANT_FIND;
        }
    }

    return REPEAT;
}
#endif

#if 0
static int Action_TracePhase3_(IR_DIRECTION* direction, int* startdirection)
{
    int result = 0;
    static int StoneCount = 50;

    if(StoneCount > 0)
    {
        Move_Forward(DEFAULT_SPEED);
        StoneCount--;
    }
    else
    {
        result = Action_FindStation(direction, FIND_SHORT);
        debug.detectlev = result;
        *startdirection = result;

        if(result == FIND_CENTER || result == FIND_LEFT || result == FIND_RIGHT)
        {
            StoneCount = 10;
            //return MODE_TRACE_SHORT_IR;
        }
        else if(result == REPEAT)
        {   
            return MODE_FIND_SHORT_IR;
        }    
        else if(result == CANT_FIND)
        {
            StoneCount = 10;
            return MODE_FIND_LONG_IR;
            //error
        }
        else
        {

        }
    }
    
    return MODE_FIND_SHORT_IR;
}

static int Action_TracePhase4_(IR_DIRECTION* direction)
{
    static int ccount = 200;
    if(ccount > 0)
    {
        ccount--;
        Move_Forward(DEFAULT_SPEED);
    }
    else
    {
        ccount = 0;
        Stop_Robot();
        return MODE_FIND_LONG_IR;
    }
}
#endif

#if 0
static int Action_TracePhase4(IR_DIRECTION* direction, int* startdirection)
{
    static int FlagSetDirection;
    int currentmotion = 0;
    int result = 0;

    if(!FlagSetDirection)
    {
        Action_SetStartDirection(startdirection, &FlagSetDirection);
    }
    else
    {    
        result = Action_TraceShortIr(direction, &currentmotion);

        //Action_Robot(currentmotion);
        Action_SwingMove(currentmotion);
        debug.direction = currentmotion;

        if(result == SUCCESS_DETECTED)
        {
            //return MODE_TRACE_LONG_IR;
            FlagSetDirection = 0;
            return MODE_GO_TO_HOME;
        }
        else if(result == FAIL_DETECTED)
        {
            //return MODE_TRACE_LONG_IR;
            FlagSetDirection = 0;
            return MODE_FIND_LONG_IR;
            //error
        }
    }
    
    return MODE_TRACE_SHORT_IR;
}
#endif

#if 0
static void Action_SwingMove(int motion)
{
    static int FlagSwing, Step;
    static int CountSwing = SWING_TIME;
    static int CountMove = MOVE_TIME;

    switch(motion)
    {
        case FORWARD :
        {
            switch(Step)
            {
                case 0 :
                {
                    CountMove--;
                    Move_Forward(DEFAULT_SPEED);
                    if(CountMove == 0)
                    {
                        CountMove = MOVE_TIME;
                        Step++;
                    }
                    break;
                }
                case 1 :
                {
                    CountSwing--;
                    Move_Swing_Left(-DEFAULT_SPEED+3, DEFAULT_SPEED-3);
                    if(CountSwing == 0)
                    {
                        CountSwing = SWING_TIME;
                        //Step++;
                        Step = 0;
                    }
                    break;
                }
                case 2 :
                {
                    CountSwing--;
                    Move_Swing_Right(DEFAULT_SPEED-5, -DEFAULT_SPEED+5);
                    if(CountSwing == 0)
                    {
                        CountSwing = SWING_TIME;
                        Step++;
                    }
                    break;
                }
                case 3 :
                {
                    CountSwing--;
                    Move_Swing_Left(-DEFAULT_SPEED+3, DEFAULT_SPEED-3);
                    if(CountSwing == 0)
                    {
                        CountSwing = SWING_TIME;
                        Step = 0;
                    }
                    break;
                }
            }
        }
    }
}
#endif

#if 0
static int Choice_Swing_Direction(IR_DIRECTION* direction)
{
    IR_DIRECTION currentdirection = *direction;

    static int ShortCount;
    
    if(IS_DETECTED_SHORT_ONLY_CENTER_DIRECTION(currentdirection))
    {        
        ShortCount++;
        if(ShortCount >= IR_COUNT(1))
        {
            Stop_Robot();
            ShortCount = 0;
            return DETECTED;
        }
    }
    else
    {
        ShortCount = 0;
        return FORWARD;
    }
}
#endif

