#include "arduino.h"
#include "FS.h"

#define dEventQueueSize 16

#define MODE_SCRIPT        0x00
#define MODE_HOST          0x01
#define MODE_UPLOAD        0x02
//========================================================
//DEFINE STORY BOARD CONSTANT
//========================================================
//STORY BOARD BYTE INDEX
#define EVENT_CODE    0
#define EVENT_VALUE   1
#define DEVICE_ID     2
#define ACTION_CODE   3
#define OBJECT_ID     4
#define PARAM_NAME    5
#define PARAM_1       6
#define PARAM_2       7
//========================================================
//EVENT_CODE - BYTE_0
//========================================================
#define EC_INPUT_S                     0x01
#define EC_INPUT_O                     0x02
#define EC_INPUT_X                     0x04
#define EC_INPUT_SO                    0x03
#define EC_INPUT_SX                    0x05
#define EC_INPUT_OX                    0x06
#define EC_INPUT_SOX                   0x07
#define EC_INPUT_USER                  0x08
#define EC_INPUT_ANY                    0x0F
//#define EC_INPUT_VALUE                  0x20

#define EC_TIME_100MS              (byte)0x30
#define EC_TIME_1SECOND            (byte)0x31
#define EC_TIME_1MINUTE            (byte)0x32
#define EC_TIME_1HOUR              (byte)0x33

#define EC_ROUTINE                 (byte)0xA0
#define EC_FLAG_TRUE               (byte)0xA1
#define EC_FLAG_FALSE              (byte)0xA2

#define EC_STAGE_END               (byte)0xFE
#define EC_END                     (byte)0xFF
#define EC_NULL                    (byte)0x00

#define EC_CD_LEFT                 (byte)0x82
#define EC_CD_BOTTOM               (byte)0x83
#define EC_CD_RIGHT                (byte)0x84
#define EC_CD_TOP                  (byte)0x85
#define EC_CD_OBJECT               (byte)0x86
//========================================================
//EVENT_VALUE - BYTE_1
//========================================================

//========================================================
//DEVICE_ID - BYTE_2
//========================================================
#define DI_ID_0                          0x00
#define DI_ID_1                          0x01
#define DI_ID_2                          0x02
#define DI_ID_3                          0x03
#define DI_ID_4                          0x04
#define DI_ID_5                          0x05
#define DI_ID_6                          0x06
#define DI_ID_7                          0x07
#define DI_ID_8                          0x08
#define DI_ID_9                          0x09
#define DI_ID_10                         0x0A
#define DI_ID_11                         0x0B
#define DI_ID_12                         0x0C
#define DI_ID_13                         0x0D
#define DI_ID_14                         0x0E
#define DI_ID_15                         0x0F
#define DI_ID_ALL                  (byte)0xFF
#define DI_ID_ANY                  (byte)0xFF
#define DI_ID_HOST                 (byte)0xFE

//========================================================
//ACTION_CODE - BYTE_3
//========================================================
#define AC_NULL                          0x00
#define AC_OBJECT_NEW                    0x01
#define AC_OBJECT_REMOVE                 0x02
#define AC_OI_INIT                       0x03
#define AC_OI_SET                        0x04
#define AC_OI_GET_FROM                   0x05
#define AC_OI_SET_TO                     0x06
#define AC_OI_SHOOT                      0x07
//#define AC_OI_VARIABLE_GET_ID            0x08
#define AC_OI_VARIABLE_SET               0x09
#define AC_OI_VARIABLE_GET_FROM          0x0A
#define AC_OI_VARIABLE_SET_TO            0x0B

#define AC_OI_PLUS                       0x10
#define AC_OI_MINUS                      0x11
#define AC_OI_TIMES                      0x12
#define AC_OI_DIVIDE                     0x13
#define AC_OI_MOD                        0x16

#define AC_OI_ORDINAL_LOCAL              0x18
#define AC_OI_RANDOM_LOCAL               0x19
#define AC_OI_ORDINAL_FROM_EI            0x1A
#define AC_OI_ORDINAL_FROM_EI_H          0x1B
#define AC_OI_ORDINAL_FROM_EI_L          0x1C
#define AC_OI_RANDOM_FROM_EI             0x1D
#define AC_OI_RANDOM_FROM_EI_H           0x1E
#define AC_OI_RANDOM_FROM_EI_L           0x1F

#define AC_OI_EQUAL                      0x20
#define AC_OI_MORE_THAN_EQUAL            0x21
#define AC_OI_LESS_THAN_EQUAL            0x22
#define AC_OI_MORE_THAN                  0x23
#define AC_OI_LESS_THAN                  0x24
#define AC_OI_POSITION_EQUAL             0x25

#define AC_OI1_EQUAL_OI2                 0x26
#define AC_OI1_MORE_THAN_OI2             0x27
#define AC_OI1_LESS_THAN_OI2             0x28
#define AC_OI1_MORE_THAN_EQUAL_OI2       0x29
#define AC_OI1_LESS_THAN_EQUAL_OI2       0x2A
#define AC_OI1_POSITION_EQUAL_OI2        0x2B
#define AC_OIV1_EQUAL_OIV2               0x2C

#define AC_OI1_PLUS_EQUAL_OI2            0x30
#define AC_OI1_MINUS_EQUAL_OI2           0x31
#define AC_OI1_TIMES_EQUAL_OI2           0x32
#define AC_OI1_DIVIDE_EQUAL_OI2          0x33
#define AC_OI1_MOD_EQUAL_OI2             0x34
#define AC_RESULT_EQUAL_OI1_PLUS_OI2     0x35
#define AC_RESULT_EQUAL_OI1_MINUS_OI2    0x36
#define AC_RESULT_EQUAL_OI1_TIMES_OI2    0x37
#define AC_RESULT_EQUAL_OI1_DIVIDE_OI2   0x38
#define AC_RESULT_EQUAL_OI1_MOD_OI2      0x39

#define AC_OBJECT_SYNC                   0x61

#define AC_STAGE_GOTO              (byte)0x91
//========================================================
//OBJECT_ID - BYTE_4
//========================================================
#define OI_0                             0x00
#define OI_1                             0x01
#define OI_2                             0x02
#define OI_3                             0x03
#define OI_4                             0x04
#define OI_5                             0x05
#define OI_6                             0x06
#define OI_7                             0x07
#define OI_8                             0x08
#define OI_9                             0x09
#define OI_10                            0x0A
#define OI_11                            0x0B
#define OI_12                            0x0C
#define OI_13                            0x0D
#define OI_14                            0x0E
#define OI_15                            0x0F

#define OI_ALL                     (byte)0xFF //OI_0~OI_15
#define OI_SELECTED                (byte)0xFE //select from OI_0~OI_15
//#define OI_MYID                    (byte)0xFD //hardware device id
//#define OI_EVENTID                 (byte)0xFC //EVENT input device id
#define OI_TRIGGER                 (byte)0xFD

#define OI_PASS                    (byte)0x80
#define OI_BOUNDARY                (byte)0x81
#define OI_BOUNDARY_L              (byte)0x82 //for collision check
#define OI_BOUNDARY_B              (byte)0x83 //for collision check
#define OI_BOUNDARY_R              (byte)0x84 //for collision check
#define OI_BOUNDARY_T              (byte)0x85 //for collision check

#define OI_VARIABLE                      0x10
#define OI_LED8x8                        0x20
#define OI_LED_RGB                       0x21
#define OI_MOTOR                         0x30
//========================================================
//PARAM_NAME - BYTE_5
//========================================================
#define PN_PASS                    (byte)0x80

//OI_0~OI_15 attribute
#define PN_VALUE                         0x00
#define PN_X                             0x01
#define PN_Y                             0x02
#define PN_XDEST                         0x03
#define PN_YDEST                         0x04
#define PN_XVEL                          0x05
#define PN_YVEL                          0x06
#define PN_DIR                           0x07
#define PN_WIDTH                         0x08
#define PN_HEIGHT                        0x09
#define PN_CD                            0x0A
#define PN_IMAGE                         0x0B
#define PN_AT                            0x0C
#define PN_XOFFSET                       0x0D
#define PN_YOFFSET                       0x0E
//with param1 , param2
#define PN_POSITION                      0x0F
#define PN_XYOFFSET                      0x10
#define PN_XYDEST                        0x11
#define PN_XYOFFSETDEST                  0x12
#define PN_SIZE                          0x13

//OI_TRIGGER attribute
#define PN_EVENT_CODE                    0x00
#define PN_EVENT_VALUE                   0x01
#define PN_DEVICE_ID                     0x02
#define PN_MYID                          0x03
#define PN_SELECTED                      0x04
#define PN_STATE_VALUE_1                 0x05
#define PN_STATE_VALUE_2                 0x06
#define PN_STATE_VALUE_3                 0x07
#define PN_STATE_VALUE_4                 0x08
#define PN_STATE_VALUE_5                 0x09
#define PN_STATE_VALUE_6                 0x0A
//========================================================
//PARAM_1 - BYTE_5
//========================================================
//OI_0~OI_15 AC_OI_SHOOT param
#define P1_DIR_RANDOM             (byte)0x81
#define P1_DIR_ORDINAL            (byte)0x82
//OI_0~OI_15 PN_CD param
#define PN_CD_NONE                (byte)0x00
#define PN_CD_INVERT_VXorVY       (byte)0x01     
#define PN_CD_INVERT_VX_VY        (byte)0x02     
#define PN_CD_INVERT_VX_VY0       (byte)0x03     
#define PN_CD_INVERT_VY_VX0       (byte)0x04     
#define PN_CD_VX0_VY0             (byte)0x05     
#define PN_CD_INVERT_VXorVY_OB    (byte)0x06     
#define PN_CD_RANDOM              (byte)0x07     
#define PN_CD_GRAVITY             (byte)0x08     

#define PN_CD_VALUE_A_ZERO        (byte)0x10
#define PN_CD_VALUE_A_MINUS_B     (byte)0x20
#define PN_CD_VALUE_A_PLUS_B      (byte)0x30
#define PN_CD_VALUE_B_ZERO        (byte)0x40
#define PN_CD_VALUE_B_MINUS_A_A0  (byte)0x50
#define PN_CD_VALUE_B_PLUS_A_A0   (byte)0x60
#define PN_CD_VALUE_AB_MINUS_BA   (byte)0x70
#define PN_CD_VALUE_FIXED         (byte)0x80

//OI_BOUNDARY attribute
#define PN_BOUNDARY_LEFT                0x00
#define PN_BOUNDARY_BOTTOM              0x01
#define PN_BOUNDARY_RIGHT               0x02
#define PN_BOUNDARY_TOP                 0x03
#define PN_BOUNDARY_CD                  0x04
    //OI_BOUNDARY.PN_BOUNDARY_CD param
#define PN_BOUNDARY_CD_LEFT_LINK_RIGHT   0x01
#define PN_BOUNDARY_CD_RIGHT_LINK_LEFT   0x02
#define PN_BOUNDARY_CD_BOTTOM_LINK_TOP   0x04
#define PN_BOUNDARY_CD_TOP_LINK_BOTTOM   0x08

//OI_VARIABLE attribute
#define PN_VARIABLE_RESULT 0x00
#define PN_VARIABLE_U      0x01
#define PN_VARIABLE_V      0x02
#define PN_VARIABLE_W      0x03
#define PN_VARIABLE_X      0x04
#define PN_VARIABLE_Y      0x05
#define PN_VARIABLE_Z      0x06  

//OI_LED8x8 attribute
#define PN_LED8x8_L0              0x00
#define PN_LED8x8_L1              0x01
#define PN_LED8x8_L2              0x02
#define PN_LED8x8_L3              0x03
#define PN_LED8x8_L4              0x04
#define PN_LED8x8_L5              0x05
#define PN_LED8x8_L6              0x06
#define PN_LED8x8_L7              0x07
#define PN_LED8x8_MODE            0x08
#define PN_LED8x8_DIR             0x09
#define PN_LED8x8_VALUE           0x0A
#define PN_LED8x8_STYLE           0x0B
    //PN_LED8x8_STYLE param
#define PN_LED8x8_STYLE_CLEAR     0x00
#define PN_LED8x8_STYLE_M1_U      0x01
#define PN_LED8x8_STYLE_2_U_DEC   0x02
#define PN_LED8x8_STYLE_2_DEC     0x03
#define PN_LED8x8_STYLE_L1_DEC    0x04
#define PN_LED8x8_STYLE_R1_DEC    0x05
#define PN_LED8x8_STYLE_2_UX_DEC  0x06
#define PN_LED8x8_STYLE_2_U_HEX   0x07
#define PN_LED8x8_STYLE_L1        0x08
#define PN_LED8x8_STYLE_R1        0x09
#define PN_LED8x8_STYLE_2_HEX     0x0A
//========================================================
#define DIRECTION_0                      0x00
#define DIRECTION_1                      0x01
#define DIRECTION_3                      0x03
#define DIRECTION_5                      0x05
#define DIRECTION_6                      0x06
#define DIRECTION_7                      0x07
#define DIRECTION_9                      0x09
#define DIRECTION_11                     0x0B
#define DIRECTION_12                     0x0C
#define DIRECTION_TURN_RIGHT             0x02
#define DIRECTION_TURN_LEFT              0x0A
//========================================================
#define P1_SYMBOL_QUESTION                16
#define P1_SYMBOL_PLUS                    17
#define P1_SYMBOL_MINUS                   18
#define P1_SYMBOL_TIMES                   19
#define P1_SYMBOL_DIVIDE                  20
#define P1_SYMBOL_EQUAL                   21
#define P1_SYMBOL_LESS                    22
#define P1_SYMBOL_MORE                    23
//========================================================
#define AT00000000B       0
#define AT00000001B       1
#define AT01010101B       2 
#define AT00110011B       3
#define AT00001111B       4
#define AT01210121B       5
#define AT01020102B       6
#define AT12312312B       7
#define AT13121312B       8
#define AT23432343B       9
#define AT56765676B       10
#define AT01230123B       11
#define AT03210321B       12
#define AT12341234B       13
#define AT45674567B       14
#define AT01234567B       15
//========================================================
#define cmdSetBrightness        0x04
#define cmdSetAttribute         0x05
//#define cmdSetBitmap1s          0x06
#define cmdSetMode              0x07
#define cmdSetID                0x09
#define cmdInputState           0x10
#define cmdStoryBoardErase      0x20
#define cmdStoryBoardWrite      0x21
#define cmdSpriteErase          0x22
#define cmdSpriteWrite          0x23
//#define cmdTimeSync           0x60
#define cmdObjectSync           0x61
#define cmdObjectSet            0x62

#define cmdRequestID            0x19
//========================================================
#define dRecvSize                136
#define dSendSize                136
#define dObjectLimit              16
#define dBypassValue    (int8_t)0x80
#define dBypassImage      (byte)0xFF
#define dBypassAnimate    (byte)0xFF


#define dTaskRunIntervalMax    20000
#define dTaskRunIntervalMin       50

typedef void (*CALLBACK) (byte,byte,byte);

class IOCUBE
{  
	public:		

    IOCUBE();      
    ~IOCUBE(); 

    void ble_server_on();
    
    void begin();
    void setCallback(CALLBACK CB);

    void drawBitmap(byte bId,const byte aBitmap[]);
    void setMode(byte bMode);
    void setBrightness(byte bValue);
    static void sendChar(byte bChar);
    static void sendSum();
        
	private:
    static void ble_check();
    
    static byte st_bMode;
  
    static CALLBACK eventCallback;
    static const byte *st_ptSprite;
    static const byte *st_ptStoryboard;
    static const byte *st_ptAnimate;
    static byte m_at_index;

    static void eventTrigger();
    
    static byte stEventSet;
    static byte stEventGet;
    static byte stEventCode[];
    static byte stEventValue[];
    static byte stEventID[];
    static void stDispatchEvent(byte bEventCode,byte bEventValue,byte bEventId);
    static uint16_t stTaskRunInterval;
    static uint16_t timeCount;

    static byte stageLoopStart;
    static byte oiSelected;
    static byte m_ordinal;
    static byte m_random;  
    static byte m_state[];
    
    static void stSerial(); 
    static byte stSerialIndex;
    static byte stRecvBuffer[];
    static byte stRecvIndex;
    static byte stSendBuffer[];
    static byte stSendIndex;
    
    static void taskSerial(void *pvParameters);
    static void taskRun(void *pvParameters);

    static void appInit();
    static void object_init(byte bId);
    static byte object_new();
    static byte object_remove(byte bId);
    static void object_update_xy();
    static void object_cd_object();
    static byte object_pixel_collision(byte A,byte B);
    static byte object_zero_draw(byte i,byte input_x,byte input_y,byte mIndex);
    static void object_cd_boundary();
    static void object_cd_behavior(byte bId,byte bIdB);
    static void object_update_to_map();
    static void object_measure_size(unsigned char bId,unsigned char bOffset);
 
    static void object_set_direction(byte bId,byte vValue);
    static void object_set_velocity(byte bId,int8_t i8X,int8_t i8Y);
    static byte object_check_direction(byte bId);
    static void appScriptEngine(byte eventCode,byte eventValue,byte inputID);
    static void doStoryboardCheckID(byte index);
    static byte getStoryboardByte(byte index,byte offset);
    static void doStoryboardAction(byte index);
    static void doObjectSet(byte bId,byte param_name,byte param_1,byte param_2);
    static byte doObjectGet(byte bId,byte param_name);  
    static void object_sync_send();
    static void time_sync_send();
    static void logic_check(byte index,byte logic,int8_t value1,int8_t value2);
    static int8_t math_caculate(byte operatorX,int8_t value1,int8_t value2);
    static void randomPairs(byte bMin,byte bMax);

    static byte m_event_code;
    static byte m_event_value;
    static byte m_device_id;
    static byte myId;
    
    static void cap1293_read();
    static byte cap1293_old;
    static byte cap1293_count[];

    static byte m_script[];
    static byte m_sprite[];
    static void deleteFile(fs::FS &fs, const char * path);
    static void writeFile(fs::FS &fs,const char * path,unsigned char * inputData);
    static void readFile(fs::FS &fs, const char * path,byte * outputBuf);

    //auto unlock collision object
    //static byte m_cd_now;
    //static byte m_cd_last;
    //static byte m_cd_count;
    static byte matrixA[];
    static byte matrixB[];
    static int8_t dirCount[];

    //static byte m_map[];
    static byte m_sum;
    static byte m_boundary_cd;
    static bool m_flag;
    static int8_t m_left;
    static int8_t m_bottom;
    static int8_t m_right;
    static int8_t m_top;
    static int8_t m_result;
    static int8_t m_u;
    static int8_t m_v;
    static int8_t m_w;
    static int8_t m_x;
    static int8_t m_y;
    static int8_t m_z;
    static int8_t m_object_x[];       //now X
    static int8_t m_object_y[];       //now Y
    static int8_t m_object_xDest[];   //destination X
    static int8_t m_object_yDest[];   //destination Y
    static int8_t m_object_xVel[];    //velocity_x
    static int8_t m_object_yVel[];    //velocity_y
    static int8_t m_object_value[];   //value

    static byte m_object_dir[];       //direction
    static byte m_object_width[];     //width
    static byte m_object_height[];    //height    
    static byte m_object_cd[];        //collision dectection
    static byte m_object_image[];     //image
    static byte m_object_at[];        //animate
};
