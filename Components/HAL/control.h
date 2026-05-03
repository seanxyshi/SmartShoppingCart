#include "stm32f10x.h"
#include "OSAL_Clock.h"


/***************************************************************************************************
 * 궨
 ***************************************************************************************************/
#define BUFFER_SIZE 100      //
// TOF  OLEDӲIIC ֻѡһ
#define OLED_OR_TOF    0//0 ֻ OLED    1 ֻʹ TOF

typedef enum {

    FULL_ON      = 0,  // ״̬
        //λ
    FRONT        = 1,  // ǰ    1
    BACK,              //     2
        SIDE,              //       3
    LEFT_SIDE,         //     4
    RIGHT_SIDE,        //     5
    FRONT_HALF,        // ǰ벿    6
    REAR_HALF,         //     7

        //λ
    TAG_FRONT,         // ǰ                                          8
    TAG_BACK,          //                                           9
    TAG_LEFT_FRONT,    // ǰ LEFT_SIDE  FRONT_HALF Ľ   10
    TAG_LEFT_BACK,     // 󷽣 LEFT_SIDE  REAR_HALF Ľ    11
    TAG_RIGHT_FRONT,   // ǰ RIGHT_SIDE  FRONT_HALF Ľ  12
    TAG_RIGHT_BACK,    // 󷽣 RIGHT_SIDE  REAR_HALF Ľ   13
} Tag_Position;//λ λ

typedef enum {
        NO_ACTION = 0,          // ִ
    RCSF = 1,               //ң        1
    FOLLOW_STRAIGHT,        //ֱ        2
    STOP_MOTOR,             //ֹͣ            3
    BACKWARD,               //            4
    SPOT_LEFT_TURN,         // ԭ       5
    SPOT_RIGHT_TURN,        // ԭ       6
    LEFT_TURN,              //ת            7
    RIGHT_TURN,             //ת            8
    FOLLOW_CAR,             //            9
    LEFT_BACKWARD,          //          10
    RIGHT_BACKWARD,         //          11
} TurnDirection;//תָö

typedef enum {
    NO_FULL = 0,          //              0
    Lock_mode,            //ģʽ          1
    Follow_mode,          //ģʽ        2
    Recall_mode,          //ʽ        3
    Remote_mode,          //ңʽ        4
} CarMode;//ģʽ ģʽֻΪģʽ

typedef struct {
    float x;  // ״̬
    float p;  //
    float q;  // Эֵ0.01-0.1
    float r;  // Эֵ0.5-2.0
} KalmanState;
/***************************************************************************************************
 *
 ***************************************************************************************************/
#pragma pack(push, 1)


typedef struct
{
        uint32_t is_lowbattery:1;                       //
        uint32_t is_alarm:1;                            //
        uint32_t is_chrg:1;                                     //
        uint32_t is_tdby:1;                                     //
        uint32_t battery_val:10;                        //350=3.50V
        uint32_t is_offset_range_zero_bit:1;//Уλ
        uint32_t is_offset_pdoa_zero_bit:1;     //Ƕλ

        uint32_t turn_up:1;                                     //(ң)ǰ
        uint32_t turn_down:1;                           //(ң)
        uint32_t turn_left:1;                           //(ң)
        uint32_t turn_right:1;                      //(ң)
        uint32_t mode:3;                                        //(ң)ģʽ
        uint32_t recal:1;                                       //(ң)
        uint32_t lock:1;                                        //(ң)

        uint32_t dev_type:3;                            //(0:ѧϰ 1: 2:ң)
        uint32_t reserve:4;                                     //Ԥ
}Aoa_Detail_Para_t;//ң

typedef struct
{
        int      Angle_filter;    //Ƕ
        int      Distance_filter; //

        struct{
                int16_t angle;                                  //Ƕ()
                uint16_t range;                                 //(cm)
        }tag_tof_Ax[4];//AOAȵ

        struct{
                int16_t angle;                                  //Ƕ()
                uint16_t range;                                 //(cm)
                int16_t  rssi;                                  //
        }More_tag_tof_Ax[4];//AOA ȵ

        Aoa_Detail_Para_t  Aoa_para_t;//ңز

        uint8_t Remote_control;//ңذ һ
        uint8_t Car_mode;      //ңʽ

        uint8_t Tag_LR;//վ  ϳҪת
        uint8_t Tof_Directions;   //

        uint16_t Car_Speed;

}AOA_DATA;
#pragma pack(pop)

extern AOA_DATA AVG;

/***************************************************************************************************
 * ƺ
 ***************************************************************************************************/
//void follow_car_task(void);
void Read_AoA_Control(void);
void car_motor_speed (int distance);

int AOA_Angle_difference(int Angle);
int AOA_final_angle (int Angle_difference,int Angle );
int car_distance (int AOA_distance);

// ɨǹغ
void Barcode_ProcessAndEcho(void);