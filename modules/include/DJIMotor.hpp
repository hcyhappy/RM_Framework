#ifndef DJIMOTOR_HPP
#define DJIMOTOR_HPP

#include "can.h"
#include "pid.hpp"

enum GearBox {
    GearBox_None = 0,  ///< 无减速箱
    GearBox_M2006 = 1, ///< M2006减速箱
    GearBox_M3508 = 2, ///< M3508减速箱
    GearBox_XRoll = 3  ///< XRoll减速箱
};

/**
 * @class DJIMotor
 * @brief 电机控制类，提供电机的基本控制功能。
 *
 * 保留速度、位置控制所需的状态；控制计算由派生类完成。
 */
class DJIMotor {
  public:
    /**
     * @enum MotorControlModeType
     * @brief 描述电机的不同控制模式。
     */
    enum MotorControlModeType {
        RELAX_MODE = 0, ///< 电机松开模式，所有输出均为0。
        SPD_MODE = 1,   ///< 速度模式，控制电机速度。
        POS_MODE = 2    ///< 位置模式，控制电机到特定位置。
    };

    enum MotorStateTypedef { MOTOR_OFFLINE = 0, MOTOR_ONLINE = 1 };

    /**
     * @struct MotorFeedBack
     * @brief 电机反馈数据的结构体，包括电机的各种物理量反馈。
     * 结构体中包含了电机的电流、速度、位置等信息，以及电机的温度等状态反馈。
     */
    struct MotorFeedBack {
        int16_t last_ecd;      ///< 上次电机编码器的读数
        uint16_t ecd;          ///< 当前电机编码器的读数
        int16_t speed_rpm;     ///< 电机的转速，单位rpm
        float currentFdb;      // C610 raw feedback units; manual does not define A/LSB    ///<
                               // 电机电流反馈
        float speedFdb;        ///< 电机当前速度反馈, 单位rad/s
        float lastSpeedFdb;    ///< 上次记录的电机速度
        float positionFdb;     ///< 电机当前位置反馈
        float lastPositionFdb; ///< 上次记录的电机位置
        float
            temperatureFdb; // C610 has no temperature field; NaN after feedback  ///< 电机温度反馈
    };

    MotorStateTypedef MotorState;
    MotorControlModeType controlMode; ///< 当前电机控制模式
    MotorFeedBack motorFeedback;      ///< 处理后的电机的反馈数据
    uint16_t canId;                   ///< 电机的CAN通信ID
    CAN_HandleTypeDef *hcan;          ///< 指向电机使用的CAN接口的指针

    PID speedPid = PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, PID_POSITION);    ///< TODO: 配置速度环参数
    PID positionPid = PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, PID_POSITION); ///< TODO: 配置位置环参数

    float speedSet;    ///< 设定的目标速度
    float positionSet; ///< 设定的目标位置，输出轴相对多圈角度 rad

    int16_t currentSet;  ///< 设定的电流输出
    uint16_t maxCurrent; ///< 最大电流限制

    uint32_t AliveFlag;
    uint32_t Pre_AliveFlag;

    GearBox gearBox;
    bool feedbackReceived = false;
    uint32_t feedbackTick = 0;
    uint32_t offlineTimeoutMs = 100;
    int64_t encoderCounts = 0;
    bool positionValid = false; // Lost feedback requires explicit re-homing.
    /**
     * @brief 纯虚函数，用于设置电机输出。
     * 必须在派生类中实现此函数。
     */
    virtual void setOutput() = 0;

    virtual MotorStateTypedef AliveCheck() = 0;

    virtual ~DJIMotor() = default;

    /**
     * @brief 构造函数
     */
    DJIMotor() {
        controlMode = RELAX_MODE;

        speedSet = 0;
        positionSet = 0;
        currentSet = 0;

        maxCurrent = 0;

        motorFeedback = {};
        canId = 0;
        hcan = nullptr;
        AliveFlag = Pre_AliveFlag = 0;
        gearBox = GearBox_None;
        MotorState = MOTOR_OFFLINE;
    }
};

#endif // DJIMOTOR_HPP
