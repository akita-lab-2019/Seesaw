/**
 *******************************************************************************
 ** 走行体の情報を取集，管理，提供する
 *******************************************************************************
 **/

#ifndef EV3_Seesaw_H_
#define EV3_Seesaw_H_

#include <Clock.h>
#include <GyroSensor.h>
#include "GuageManager.h"
#include "LineTracer.h"
#include "TailController.h"

class Seesaw
{
public:
    Seesaw(ev3api::Clock &clock,
           ev3api::GyroSensor &gyro,
           ev3api::Motor &wheel_L,
           ev3api::Motor &wheel_R,
           GuageManager *guage,
           LineTracer *line_tracer,
           TailController *tail);

    void init();
    void run();

private:
    ev3api::Clock &m_clock;
    ev3api::GyroSensor &m_gyro;
    ev3api::Motor &m_wheel_L;
    ev3api::Motor &m_wheel_R;
    GuageManager *m_guage;
    LineTracer *m_line_tracer;
    TailController *m_tail;

    void update();
    void lineRun(bool is_inverted, int forward, int pid_index, int target);
    void downBody();
    void upBody();
    void landing();

    // 現在のシーソーシーケンス番号
    int m_sequence_num = 0;

    // 着地した時の走行距離
    float m_seesaw_start_dis = 0;

    // 走行のPIDパラメータ
    float m_run_pid_param[7][3] = {{0.080, 0.000, 0.000},
                                   {0.200, 0.000, 0.000},
                                   {0.000, 0.000, 0.000},
                                   {0.100, 0.000, 0.000},
                                   {0.100, 0.000, 0.000},
                                   {0.100, 0.000, 0.000},
                                   {0.100, 0.000, 0.000}};
};

#endif
