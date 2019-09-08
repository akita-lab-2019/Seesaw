#include "Seesaw.h"

/**
 * コンストラクタ
 */
Seesaw::Seesaw(ev3api::Clock &clock,
               ev3api::GyroSensor &gyro,
               ev3api::Motor &wheel_L,
               ev3api::Motor &wheel_R,
               GuageManager *guage,
               LineTracer *line_tracer,
               TailController *tail)
    : m_clock(clock),
      m_gyro(gyro),
      m_wheel_L(wheel_L),
      m_wheel_R(wheel_R),
      m_guage(guage),
      m_line_tracer(line_tracer),
      m_tail(tail)
{
}

/**
 * 初期化する
 */
void Seesaw::init()
{
}

/**
 * 情報を更新する
 */
void Seesaw::update()
{
}

/**
 * 走行を行う
 */
long start_time = 0;
void Seesaw::run()
{
    int fwd;
    switch (m_sequence_num)
    {
    // ゴール通知
    case 0:
        ev3_speaker_play_tone(262, 500);
        m_seesaw_start_dis = m_guage->getRobotDis();
        m_line_tracer->setPidParm(m_run_pid_param[0]);
        m_sequence_num++;
        break;

    // シーソー検知まで減速
    case 1:
        // ゴールゲートから遠ざかるほど前進量を下げる
        // fwd = 100 - 150 * (m_guage->getRobotDis() - m_seesaw_start_dis);

        // // 前進量の下限は20
        // if (fwd < 20)
        fwd = 20;

        lineRun(1, fwd, 0, 35);

        // シーソーを検知
        if (abs(m_guage->getPitchVel()) > 150 && (m_guage->getRobotDis() - m_seesaw_start_dis) > 0.4)
        {
            ev3_speaker_play_tone(262, 100);
            m_seesaw_start_dis = m_guage->getRobotDis();
            m_line_tracer->setPidParm(m_run_pid_param[1]);
            m_sequence_num++;
        }
        break;

    // 後進
    case 2:
        lineRun(1, -32, 0, 35);
        if (m_guage->getRobotDis() - m_seesaw_start_dis < -0.15)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    // 前進
    case 3:
        lineRun(1, 80, 0, 35);
        if (m_guage->getRobotDis() - m_seesaw_start_dis > 0)
        {
            ev3_speaker_play_tone(262, 100);
            start_time = m_clock.now();
            m_line_tracer->setPidParm(m_run_pid_param[0]);
            m_sequence_num++;
        }
        break;

    // 前進
    case 4:
        m_gyro.setOffset(-31);
        lineRun(1, 90, 0, 35);
        if (m_clock.now() - start_time > 200)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    // 前進
    case 5:
        m_gyro.setOffset(0);
        lineRun(1, 90, 0, 35);
        if (m_guage->getRobotDis() - m_seesaw_start_dis > 0.10)
        {
            ev3_speaker_play_tone(262, 100);
            start_time = m_clock.now();
            m_sequence_num++;
        }
        break;

    // 前進
    case 6:
        lineRun(1, 0, 0, 35);
        if (m_clock.now() - start_time > 700)
        {
            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    // 前進
    case 7:
        lineRun(1, 20, 0, 35);
        break;
    default:
        break;
    }
}

void Seesaw::upBody()
{
    m_tail->setAngle(90);
    m_tail->setMaxSpeed(100);
}

void Seesaw::downBody()
{
    m_tail->setAngle(70);
    m_tail->setMaxSpeed(100);
}

/**
 * ライントレースする
 *
 */
void Seesaw::lineRun(bool is_inverted, int forward, int pid_index, int target)
{
    m_line_tracer->setIsInverted(is_inverted);
    m_line_tracer->setForward(forward);
    m_line_tracer->setCurvature(0);
    m_line_tracer->setColorTarget(target);
    m_line_tracer->run();
}

void Seesaw::landing()
{
}
