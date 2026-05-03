#include "control.h"
#include "uwb.h"
#include "hal_usart.h"
#include "math.h"
#include "delay.h"
#include "oled_i2c.h"
#include "tof_i2c.h"
#include "motor.h"
#include "Periph_init.h"
#include "hal_iic.h"
#include "uart4_process.h"
TOF_Distance TOF_D;
AOA_DATA AVG={0};

/**************************************************************************
  Ƕȴ
ϴǶ
ڲcurrent_angle  ҪĽǶ   last_angle    һνǶ
          output_num              output_thresh
          inverse_thresh Ǳ仯   same_thresh   ͬǱ仯
  ֵǶ
**************************************************************************/
static void process_single_angle(int16_t* current_angle, int* last_angle,
                                                                int* output_num,uint8_t output_thresh,
                                                                int inverse_thresh, int same_thresh) {
        int curr = *current_angle;
        int last = *last_angle;
        int outnum=*output_num;

        // ʽǶȲԲ
        int delta = curr - last;

        // ϽǶȱ仯
        int is_same_direction = (last * curr) >= 0;  // ͬ

        // Ӧ̬
        int threshold = is_same_direction ? same_thresh : inverse_thresh;

        // 䳬
        if(delta == curr && last==0)//νʱ
        {//Ϊ current_angle  Ϊ0ģοʼʱ
         //Żᴥ
                *current_angle=curr; // ֵ
                *last_angle = curr;  // ʷֵ
                *output_num=0;
        }
        else if(outnum>=output_thresh)//
        {
                *current_angle=curr; // ֵ
                *last_angle = curr;  // ʷֵ
                *output_num=0;
        }
        else if(abs(delta) > threshold)// ʹ
        {
                *current_angle = *last_angle;  // ϴֵ
                (*output_num)++;
        }
        else //δ ʹ
        {
                *current_angle=curr; // ֵ
                *last_angle = curr;  // ʷֵ
        }
}

/**************************************************************************
  AOA_Angle_Filter
ϴǶȣݵĻվ
ڲAVG ҪĽṹ
          inverse_thresh  Ǳ仯
          same_thresh     ͬǱ仯
          output_thresh
  ֵǶ
**************************************************************************/
void AOA_Angle_Filter(AOA_DATA* AVG, int inverse_thresh, int same_thresh,
                      uint8_t output_thresh)
{
    static int last_angles[4] = {0}; // 洢4վʷǶ
    static int output_num[4] = {0};  // 洢ǰ4վ

    for(int i=0; i<4; i++)
        {
#ifdef USE_RSSI_TAG //Ƿźȣѡ
                //ȷMore_tag_tof_Ax
                if(AVG->More_tag_tof_Ax[i].angle !=0 && AVG->More_tag_tof_Ax[i].range !=0)
                {
                        process_single_angle(&AVG->More_tag_tof_Ax[i].angle,&last_angles[i],
                                                                &output_num[i],output_thresh,
                                                                inverse_thresh,
                                                                same_thresh);
                }
#else
                //ȷtag_tof_Ax
                if(AVG->tag_tof_Ax[i].angle !=0 && AVG->tag_tof_Ax[i].range !=0)
                {
                        process_single_angle(&AVG->tag_tof_Ax[i].angle,&last_angles[i],
                                                                &output_num[i],output_thresh,
                                                                inverse_thresh,
                                                                same_thresh);
                }
#endif
        }
}

/**************************************************************************
  Kalman_filtering
ĿݵĻվ
ڲAVG       ݴ洢λ
  ֵ
:
**************************************************************************/
void kalman_filter(AOA_DATA *AVG)
{
        static KalmanState angle_states[4] = {0};//Ƕ˲Ŀ
        static KalmanState range_states[4] = {0};//Ŀ

        for (int i = 0; i < 4; ++i) {
                // ʼֵ
                int16_t meas_angle; uint16_t meas_range;

        #ifdef USE_RSSI_TAG //Ƿźȣѡ
                meas_angle = AVG->More_tag_tof_Ax[i].angle;
                meas_range = AVG->More_tag_tof_Ax[i].range;
        #else
                meas_angle = AVG->tag_tof_Ax[i].angle;
                meas_range = AVG->tag_tof_Ax[i].range;
        #endif

                // Ƕ˲
                if (angle_states[i].p == 0) { // γʼ
                        angle_states[i].x = (float)meas_angle;
                        angle_states[i].p = 1.0f;
                        angle_states[i].q = 0.4f;  //
                        angle_states[i].r = 0.7f;  //
                } else {
                        // Ԥ
                        angle_states[i].p += angle_states[i].q;

                        //
                        float k = angle_states[i].p / (angle_states[i].p + angle_states[i].r);
                        angle_states[i].x += k * ((float)meas_angle - angle_states[i].x);
                        angle_states[i].p *= (1 - k);
                }

                //
                if (range_states[i].p == 0) { // γʼ
                        range_states[i].x = (float)meas_range;
                        range_states[i].p = 1.0f;
                        range_states[i].q = 0.3f;
                        range_states[i].r = 0.5f;
                } else {
                        // Ԥ
                        range_states[i].p += range_states[i].q;

                        //
                        float k = range_states[i].p / (range_states[i].p + range_states[i].r);
                        range_states[i].x += k * ((float)meas_range - range_states[i].x);
                        range_states[i].p *= (1 - k);
                }

                //
        #ifdef USE_RSSI_TAG
                AVG->More_tag_tof_Ax[i].angle = (int16_t)roundf(angle_states[i].x);
                AVG->More_tag_tof_Ax[i].range = (uint16_t)roundf(range_states[i].x);
        #else
                AVG->tag_tof_Ax[i].angle = (int16_t)roundf(angle_states[i].x);
                AVG->tag_tof_Ax[i].range = (uint16_t)roundf(range_states[i].x);
        #endif
        }
}

/**************************************************************************
  TOF10120_Judgment
ݵǰֵǷ
                  0~25Ϊʽ  ʽͽ
ڲ
  ֵint ֵ
**************************************************************************/

int TOF10120_Judgment (float T_L,float T_ML,float T_MR,float T_R)//
{
        //ʽλ     -3    L -2   I -1    0   J 1   K2    3
  static int Car_mode,Car_number=0,car_move=NO_ACTION,turn_number,turn_move=0,move_time=0;
        //Car_number ϰ
        //car_move ȷַʽ
        //turn_move ֹص
        //move_time 5ڵı
        int  Barrier=0,Car_status=(T_L+T_ML)-(T_MR+T_R);//
        //  Ͼ䷶Χϰ
        if (T_L<37 || T_ML<37 || T_MR<37 || T_R<37)//3ǰ 25  25  25  25
        {if(Car_number>=0){Car_number=0;Car_number--;}Car_number--;}
        else if(T_L>=37 && T_ML>=37 && T_MR>=37  && T_R>=37){if(Car_number<=0){Car_number=0;Car_number++;}Car_number++;}// 25 25 25 25
        if(Car_number<=-3){Car_number=0;Car_mode=1;}//ʽ
        else if(Car_number>=3 && car_move==NO_ACTION){Car_number=0;Car_mode=0;}//ʽ

  // ϳ
        if(T_L<10){Barrier++;}if(T_ML<=15){Barrier++;}if(T_MR<=15){Barrier++;}if(T_R<10){Barrier++;}
        if(Barrier==4){turn_number=1;}//ֱӱܹϰ
        else if(Barrier<=0){turn_number=0;turn_move=0;}//ǰʱ

        if(Car_mode==1 && turn_move==0)
        {
                if(turn_number==1)//Ҫ
                {
                        if(Car_status<0){car_move=SPOT_LEFT_TURN;}//
                        else if(Car_status>=0){car_move=SPOT_RIGHT_TURN;}//
                }
                else if(T_L<14 || T_ML<12 || T_MR<12 || T_R<14)//Ҫʱ// 12  10  10  12
                {
                        if(Car_status<0){car_move=LEFT_BACKWARD;}//󵹳
                        else if(Car_status>=0){car_move=RIGHT_BACKWARD;}//󵹳
                        move_time=3;
                }
                else if(T_L<19 || T_ML<22 || T_MR<22 || T_R<19)//Ҫתʱ17  20  20  17
                {
                        if(Car_status<0){car_move=LEFT_BACKWARD;}//ת
                        else if(Car_status>=0){car_move=RIGHT_BACKWARD;}//ת
                        move_time=3;
                }
                else if(move_time<=0){car_move=0;}//
        }
        if(move_time>=0){move_time--;}//תʱֹ߽ײ
        if(car_move==SPOT_LEFT_TURN || car_move==SPOT_RIGHT_TURN){turn_move=1;}//˶ֹ
        //_dbg_printf("/car_move:%d\n",car_move);
        return car_move;
}

/**************************************************************************
  AOA_Control
AOAңؿ
ڲaoa  յAOA
  ֵint ֵ Ʋ
**************************************************************************/
int AOA_Control(AOA_DATA *AVG)
{
         uint8_t return_bit=0;

         //Ƿʽ
         if(AVG->Aoa_para_t.mode==1 && AVG->Aoa_para_t.recal==0 )//ңؿ
         {
                        return_bit=STOP_MOTOR;//Ч ֹͣ
                        if(AVG->Aoa_para_t.turn_up==1){return_bit=RCSF;}//ǰ
                        else if(AVG->Aoa_para_t.turn_down==1){return_bit=BACKWARD;}//
                        else if(AVG->Aoa_para_t.turn_left==1){return_bit=SPOT_LEFT_TURN;}//
                        else if(AVG->Aoa_para_t.turn_right==1){return_bit=SPOT_RIGHT_TURN;}//
         }

//       _dbg_printf("lock:%d  recal:%d  mode:%d  turn_right:%d  turn_left:%d  turn_down:%d  turn_up:%d  return_bit:%d\n",AVG->Aoa_para_t.lock,AVG->Aoa_para_t.recal,
//                                                                                                                                                                                                       AVG->Aoa_para_t.mode,AVG->Aoa_para_t.turn_right,
//                                                                                                                                                                                                       AVG->Aoa_para_t.turn_left,AVG->Aoa_para_t.turn_down,
//                                                                                                                                                                                                       AVG->Aoa_para_t.turn_up,return_bit);

         return return_bit;
}


/**************************************************************************
  Oled_And_Tof_Control
OLEDȡ  TOFݶȡӦ
ڲAOAǶ  ңʽ
  ֵTOFϽ
עTOFOLEDӲͬʱʹӲIIC٣TOFʹģIICȡһ
        ģIICOLEDӲIICȡTOF
**************************************************************************/
int Oled_And_Tof_Control(int Aoa_Ang,int Aoa_Dis,uint8_t YK_mode)
{
        static TOF_Distance prev_tof_data = {0}; // 洢һεTOFڱȽ
        int16_t Tof_bit=0;

        //OLED
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_DeInit(I2C2);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,DISABLE);
        IIC_Init();     //IIC
        OLED_ALL_Display(YK_mode);
        OLED_Follow_Data_Display(Aoa_Dis,Aoa_Ang);

        //ӲTOFӦ
        I2C2_Configuration();//ӲIIC
        TOF250_READ(&TOF_D);

        // ˲ֵӰ
        TOF_Distance filtered_data;

        // ݽ
        filtered_data.LEFT_MM = (TOF_D.LEFT_MM > 0 && TOF_D.LEFT_MM <= 200) ? TOF_D.LEFT_MM :
                                                        (prev_tof_data.LEFT_MM > 0 ? prev_tof_data.LEFT_MM : 100);
        filtered_data.MIDDLE_L_MM = (TOF_D.MIDDLE_L_MM > 0 && TOF_D.MIDDLE_L_MM <= 200) ? TOF_D.MIDDLE_L_MM :
                                                                (prev_tof_data.MIDDLE_L_MM > 0 ? prev_tof_data.MIDDLE_L_MM : 100);
        filtered_data.MIDDLE_R_MM = (TOF_D.MIDDLE_R_MM > 0 && TOF_D.MIDDLE_R_MM <= 200) ? TOF_D.MIDDLE_R_MM :
                                                                (prev_tof_data.MIDDLE_R_MM > 0 ? prev_tof_data.MIDDLE_R_MM : 100);
        filtered_data.RIGHT_MM = (TOF_D.RIGHT_MM > 0 && TOF_D.RIGHT_MM <= 200) ? TOF_D.RIGHT_MM :
                                                         (prev_tof_data.RIGHT_MM > 0 ? prev_tof_data.RIGHT_MM : 100);

        // ʷ
        prev_tof_data = filtered_data;

        // ʹݽ
        Tof_bit=TOF10120_Judgment(filtered_data.LEFT_MM, filtered_data.MIDDLE_L_MM, filtered_data.MIDDLE_R_MM, filtered_data.RIGHT_MM);//;

        // ǿǰ
        float front_avg = (filtered_data.MIDDLE_L_MM + filtered_data.MIDDLE_R_MM) / 2.0f;

        // ǰͻȻ
        static float prev_front_avg = 0;
        if(prev_front_avg > 0 && front_avg < prev_front_avg * 0.6 && front_avg < 30) {
                // ǰС
                Tof_bit = front_avg < (filtered_data.LEFT_MM + filtered_data.RIGHT_MM)/2.0f ?
                          LEFT_BACKWARD : RIGHT_BACKWARD;
        }
        prev_front_avg = front_avg;

        return Tof_bit;
}

/**************************************************************************
  AOA_Pattern_recognition
ģʽ
ڲAVG  յAOA
  ֵǰϵʽ
**************************************************************************/
uint8_t AOA_Pattern_recognition(AOA_DATA *AVG)
{
        static uint8_t mode=NO_FULL,mode_bit0=0,mode_bit1=0,mode_bit2=0,mode_bit3=0;

        if(AVG->Aoa_para_t.mode==1 && AVG->Aoa_para_t.recal==0 ){mode_bit0=0; mode_bit1++; mode_bit2=0; mode_bit3=0;}//ңʽ
        else if(AVG->Aoa_para_t.mode==1 && AVG->Aoa_para_t.recal==1 ){mode_bit0=0; mode_bit1=0; mode_bit2++; mode_bit3=0;}//ʽ
        else if(AVG->Aoa_para_t.mode==0 && AVG->Aoa_para_t.recal==0 ){mode_bit0=0; mode_bit1=0; mode_bit2=0; mode_bit3++;}//ģʽ

        if(AVG->Aoa_para_t.lock==1){mode=Lock_mode;}//ֹͣ
        else if(mode_bit1>=5){mode=Remote_mode;}//_dbg_printf("ң\n");}
        else if(mode_bit2>=5){mode=Recall_mode;}//_dbg_printf("\n");}
        else if(mode_bit3>=5){mode=Follow_mode;}//_dbg_printf("\n");}

        return mode;
}

/**************************************************************************
  UpdateAoaData
ȡ
ڲaoa
  ֵ
**************************************************************************/
uint8_t UpdateAoaData(General_t *aoa)
{

        #ifdef USE_RSSI_TAG //
                         // More_tag_tof_Ax0ֵ
                for (int i = 0; i < 4; i++)
                {
                        // angleΣint16_t
                        if (aoa->More_tag_tof_Ax[i].angle != 0 &&  abs(aoa->More_tag_tof_Ax[i].angle)<90) {
                                AVG.More_tag_tof_Ax[i].angle = aoa->More_tag_tof_Ax[i].angle;
                        }

                        // rangeΣuint16_t
                        if (aoa->More_tag_tof_Ax[i].range != 0) {
                                AVG.More_tag_tof_Ax[i].range = aoa->More_tag_tof_Ax[i].range;
                        }

                        // rssiΣint16_t
                        if (aoa->More_tag_tof_Ax[i].rssi != 0) {
                                AVG.More_tag_tof_Ax[i].rssi = aoa->More_tag_tof_Ax[i].rssi;
                        }
                }
        #else  //
                for (int i = 0; i < 4; i++)
                {
                        // angleΣint16_t
                        if (aoa->More_tag_tof_Ax[i].angle != 0) {
                                AVG.tag_tof_Ax[i].angle = aoa->tag_tof_Ax[i].angle;
                        }

                        // rangeΣuint16_t
                        if (aoa->More_tag_tof_Ax[i].range != 0) {
                                AVG.tag_tof_Ax[i].range = aoa->tag_tof_Ax[i].range;
                        }
                }
        #endif
}


/**************************************************************************
  follow_car_task
1.A0վ
          2.ݣȥ
          3.ݽϱλ
ڲ
  ֵվ
**************************************************************************/
uint8_t follow_car_task(void)
{
        Message msg;
        General_t aoa;
        //ݻ
        static uint8_t Buffer_index=0;          //
        static uint8_t data_buffer[BUFFER_SIZE] = {0}; // ݻ
        uint8_t Valid_count = 0,Valid_Bit=20;

        if(get_AOA_data(&msg) == true)
        {
                if(AOA_Car_Hex_Resolution(msg.buf,&aoa))//ݽ
                {
                        // 1
                        data_buffer[Buffer_index] = 1;  Buffer_index = (Buffer_index + 1) % BUFFER_SIZE;

                        UpdateAoaData(&aoa);//AOA Ƕ

                        AOA_Angle_Filter(&AVG,60,35,25);//Ƕ

                        kalman_filter(&AVG);//Ŀ

                        memcpy(&AVG.Aoa_para_t, &aoa.para_t, sizeof(aoa.para_t));//ң
                }
        }
        else
        {
                // 0
                data_buffer[Buffer_index] = 0;   Buffer_index = (Buffer_index + 1) % BUFFER_SIZE;
        }

//U1-AOA- 33HZ   follow_car_task 10msһΣ100HZ
//data_buffer50ݣʱ500ms
//16ݣ  10Ч
        for(uint8_t i=0; i<BUFFER_SIZE; i++){Valid_count += data_buffer[i];}//ͳݸ

        if(Valid_count >= Valid_Bit ? 1 : 0)
        {
                AVG.Remote_control=AOA_Control(&AVG);//ңذť
//
                if(AVG.Aoa_para_t.dev_type==2)//ʽ
                {
                        AVG.Car_mode=AOA_Pattern_recognition(&AVG);//ңʽ
                }
                else// ģʽ
                {
                        AVG.Car_mode=Follow_mode;
                }
//              _dbg_printf("RC:%d   CM:%d   FB:%d   BS:%d\r\n",AVG.Remote_control,AVG.Car_mode,AVG.Basic_Directions,AVG.Inactive_BS);
//              _dbg_printf("1Valid_Bit:%d   Valid_count:%d\r\n",Valid_Bit,Valid_count);
        }
        else
        {
                AVG.Car_mode=NO_FULL;//
//              _dbg_printf("2Valid_Bit:%d   Valid_count:%d\r\n",Valid_Bit,Valid_count);
        }
        return Valid_count >= Valid_Bit ? 1 : 0; // 100ms 3
}


/**************************************************************************
ȡAOAݲ
ڲ
  ֵ
    WHEELTEC
**************************************************************************/
void Read_AoA_Control(void)
{
        static uint8_t Dodge_mark;
        static uint64_t Move_Time=0,Motor_Time=0,LED_time;
        //ݴ 10ms һ
        if(portGetTickCnt()-Motor_Time>=10000)//100ms
        {
                Motor_Time=portGetTickCnt();//ʱ¼
                if(follow_car_task())
                {
                        LED_time++;
                        if(LED_time%50==0){LED2(ON);}else {LED2(OFF);}//ʱ LED2˸
                }
                else{LED2(OFF);}
        }
        //OLED 50ms
        if(portGetTickCnt()-Move_Time>=50000)//40ms ε
        {
                Move_Time=portGetTickCnt();//ʱ¼

#ifdef USE_RSSI_TAG //Ƿźȣѡ
                AVG.Distance_filter=AVG.More_tag_tof_Ax[0].range;
                AVG.Angle_filter=AVG.More_tag_tof_Ax[0].angle;
#else  //
                AVG.Distance_filter=AVG.tag_tof_Ax[0].range;
                AVG.Angle_filter=AVG.tag_tof_Ax[0].angle;
#endif

                switch(AVG.Car_mode)//ģʽѡ
                {
                        case NO_FULL://
//                              _dbg_printf("NO_FULL \r\n");
																//CAR_STOP();
                                // 初始化 UART4 接收（首次进入时）
																{
                                    static u8 uart4_init_flag = 0;
                                    if(uart4_init_flag == 0)
                                    {
                                        Uart4ReceiveInit();
                                        //MotorData.Velocity = 2500;  // 默认速度
                                        MotorData.Dis = 10;         // 转向系数
                                        uart4_init_flag = 1;
                                    }
                                }
                                // 处理 UART4 接收到的目标数据并控制跟随
                                Uart4Process();
                        break;

                        case Lock_mode://ģʽ// U1 AOA
//                              _dbg_printf("Lock_mode \r\n");
                                CAR_STOP();
                        break;

                        case Follow_mode://ģʽ
                                AVG.Car_Speed=Follow_speed;
                                Car_Pwm_Direction(120,10,AVG.Car_Speed);
//                              _dbg_printf("Follow_mode \r\n");
                        break;

                        case Recall_mode://ʽ
//                              _dbg_printf("Recall_mode \r\n");
                        break;

                        case Remote_mode://ңʽ
                                AVG.Car_Speed=Rcsf_speed;
                                Preset_State_Of_Motor(AVG.Remote_control);
//                              _dbg_printf("Remote_mode \r\n");
                        break;

                        default:break;
                }
                AVG.Tof_Directions=Oled_And_Tof_Control(AVG.Angle_filter,AVG.Distance_filter,AVG.Car_mode);
                if(AVG.Tof_Directions != 0){LED1(ON);}else {LED1(OFF);}//ǰʱLED1
        }
}