//
// Created by Administrator on 2026/1/26.
//
#include "quadrotor.h"
#include "pid.h"
#include "user_math.h"
#include "message_center.h"
#include "ins_task.h"
#include "uav_cmd.h"

static Quadrotor_t *quadrotor;
static Subscriber_t *imu_data_sub;
static Subscriber_t *uav_cmd_sub;
static IMU_Data_t *imu_recv_data;
static Uav_Cmd_Data_t uav_cmd_recv;

void Quadrotor_init()
{
    PID_Init_Config_s pitch_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s roll_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s yaw_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s pitch_rate_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s roll_rate_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s yaw_rate_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PID_Init_Config_s height_pid_config = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 300.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Integral,
        .IntegralLimit = 100.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    PIDInit(quadrotor->pitch_pid,&pitch_pid_config);
    PIDInit(quadrotor->roll_pid,&roll_pid_config);
    PIDInit(quadrotor->yaw_pid,&yaw_pid_config);
    PIDInit(quadrotor->pitch_rate_pid,&pitch_rate_pid_config);
    PIDInit(quadrotor->roll_rate_pid,&roll_rate_pid_config);
    PIDInit(quadrotor->yaw_rate_pid,&yaw_rate_pid_config);
    PIDInit(quadrotor->height_pid,&height_pid_config);
    imu_data_sub = SubRegister("imu_data",sizeof(IMU_Data_t));
    uav_cmd_sub = SubRegister("uav_cmd_data",sizeof(Uav_Cmd_Data_t));
}

static void Quadrotor_control(float roll_ref, float pitch_ref, float yaw_ref, float height_ref,
                       float roll_measure, float pitch_measure, float yaw_measure,float height_measure,
                       float roll_rate_measure, float pitch_rate_measure, float yaw_rate_measure,
                       float *roll_output, float *pitch_output, float *yaw_output, float *height_output)
{
    // 角度环
    PIDCalculate(quadrotor->roll_pid, roll_measure, roll_ref);
    PIDCalculate(quadrotor->pitch_pid, pitch_measure,pitch_ref);
    PIDCalculate(quadrotor->yaw_pid, yaw_measure,yaw_ref);

    // 角速度环
    PIDCalculate(quadrotor->roll_rate_pid, roll_rate_measure,quadrotor->roll_pid->Output);
    PIDCalculate(quadrotor->pitch_rate_pid,  pitch_rate_measure,quadrotor->pitch_pid->Output);
    PIDCalculate(quadrotor->yaw_rate_pid, yaw_rate_measure,quadrotor->yaw_pid->Output);
    PIDCalculate(quadrotor->height_pid, height_measure, height_ref);
    // 输出控制量
    *roll_output = quadrotor->roll_rate_pid->Output;
    *pitch_output = quadrotor->pitch_rate_pid->Output;
    *yaw_output = quadrotor->yaw_rate_pid->Output;
    *height_output = quadrotor->height_pid->Output;
}

/**
 * @brief X型四旋翼混控器 (Mixer)
 * @param roll_output  : 横滚 PID 输出 (Positive = Roll Right)
 * @param pitch_output : 俯仰 PID 输出 (Positive = Pitch Up/Back)
 * @param yaw_output   : 偏航 PID 输出 (Positive = Yaw Right/CW)
 * @param height_output: 高度 PID 输出 (通常作为基础油门 Base Throttle)
 * @param base_throttle: 基础油门 (如果是定高模式，该值可能为0，完全由 height_output 决定；如果是姿态模式，该值为遥控器油门杆量)
 * @param motor_throttle: 输出数组，存储 4 个电机的最终值
 */
static void Quadrotor_throttle(float *roll_output, float *pitch_output, float *yaw_output,
                        float *height_output, float base_throttle, float *motor_throttle)
{
    // 1. 确定基础推力
    // 在定高模式下，推力主要由 height_output 决定
    // 在自稳/手动模式下，推力主要由 base_throttle (遥控器油门) 决定
    // 这里我们将两者结合（视你的具体控制策略而定，通常 height_pid 输出的是油门的增量或直接量）
    float thrust = base_throttle + (*height_output);

    // 2. 获取 PID 控制量
    float r = *roll_output;
    float p = *pitch_output;
    float y = *yaw_output;

    // 3. 混控算法 (Mixer Formula) - X Configuration
    //
    // M4(CW)   M2(CCW)
    //      \   /
    //       \ /
    //       / \
    //      /   \
    // M3(CCW)  M1(CW)

    // Motor 1 (右后 RR, CW):  Roll(-), Pitch(-), Yaw(-) [CW减速则机身CW加速]
    // 修正：Yaw逻辑。若想机身向右(CW)转，需要 CCW 电机加速(反扭力右)，CW 电机减速。
    // 所以 CW 电机对应 -Yaw，CCW 电机对应 +Yaw。

    // Roll:  Right(+), Left(-) -> 想右滚，右边减速(-)，左边加速(+)
    // Pitch: Up(+), Down(-)    -> 想抬头，后边减速(-)，前边加速(+)

    // 注意：正负号取决于你的 PID 输出定义和电机位置。
    // 下面的符号基于：Roll+(右倾), Pitch+(抬头), Yaw+(右转)

    // M1: 右后 (RR)
    float m1 = thrust - r - p - y;

    // M2: 右前 (FR)
    float m2 = thrust - r + p + y;

    // M3: 左后 (RL)
    float m3 = thrust + r - p + y;

    // M4: 左前 (FL)
    float m4 = thrust + r + p - y;

    // 4. 安全限幅 (Safety Clamping)
    // 这一步非常重要！防止计算结果超出电机允许范围（比如算出负数导致电机反转或停转）
    motor_throttle[0] = constrain_float(m1, MOTOR_MIN, MOTOR_MAX); // Motor 1
    motor_throttle[1] = constrain_float(m2, MOTOR_MIN, MOTOR_MAX); // Motor 2
    motor_throttle[2] = constrain_float(m3, MOTOR_MIN, MOTOR_MAX); // Motor 3
    motor_throttle[3] = constrain_float(m4, MOTOR_MIN, MOTOR_MAX); // Motor 4
}
void Quadrotor_Task()
{
     SubCopyMessage(imu_data_sub, &imu_recv_data,0);
     SubCopyMessage(uav_cmd_sub,&uav_cmd_recv,0);
    float yaw_out = 0,pitch_out = 0,roll_out = 0,heigh_out = 0,motor_out[4] = {0,0,0,0};
     Quadrotor_control(uav_cmd_recv.roll_ref,uav_cmd_recv.pitch_ref,uav_cmd_recv.yaw_ref,0,
                        imu_recv_data->Roll,imu_recv_data->Pitch,imu_recv_data->Yaw,0,
                        imu_recv_data->Gyro[2],imu_recv_data->Gyro[1],imu_recv_data->Gyro[0],
                        &roll_out,&pitch_out,&yaw_out,&heigh_out
     );
    Quadrotor_throttle(&roll_out,&pitch_out,&yaw_out,&heigh_out,0,motor_out);
}