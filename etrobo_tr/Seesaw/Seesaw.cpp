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
    float seesaw_dis = m_guage->getRobotDis() - m_seesaw_start_dis;

    switch (m_sequence_num)
    {
    // ゴール通知
    case 0:
        m_seesaw_start_dis = m_guage->getRobotDis();
        m_line_tracer->setPidParm(m_run_pid_param[0]);
        m_sequence_num++;
        break;

    // 減速
    case 1:
        lineRun(1, 80, 0, 35);
        if (seesaw_dis > 0.25)
        {
            ev3_led_set_color(LED_ORANGE);
            ev3_speaker_play_tone(262, 100);
            start_time = m_clock.now();
            m_sequence_num++;
        }
        break;

    // 前進
    case 2:
        m_tail->setAngle(70);
        m_tail->setMaxSpeed(40);
        m_wheel_L.setPWM(90);
        m_wheel_R.setPWM(90);
        m_clock.sleep(150);

        m_guage->setOdomOffset(0);
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(1000);

        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(61);
        m_wheel_L.setPWM(-50);
        m_wheel_R.setPWM(-50);
        m_clock.sleep(140);

        m_tail->setAngle(93);
        m_tail->setMaxSpeed(90);
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(1000);

        m_sequence_num++;
        break;

    // 前進
    case 3:
        lineRun(0, 5, 1, 15);

        if (m_guage->getRobotDis() > 0.6)
        {
            ev3_speaker_play_tone(262, 100);
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 4:
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(100);
        m_wheel_L.setPWM(-20);
        m_wheel_R.setPWM(-20);
        m_clock.sleep(400); // 200 to 300

        m_tail->setAngle(93);
        m_tail->setMaxSpeed(90);
        m_wheel_L.setPWM(40);
        m_wheel_R.setPWM(40);
        m_clock.sleep(380);

        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(1000);

        m_sequence_num++;
        break;

    // 前進
    case 5:
        m_wheel_L.reset();
        m_wheel_R.reset();
        break;

    // 前進
    case 6:
        m_wheel_R.setPWM(4);
        m_wheel_L.reset();
        if (m_guage->getWheelPos(GuageManager::R) > 120)
        {
            m_sequence_num++;
        }
        break;

    // 前進
    case 7:
        m_wheel_R.setPWM(4);
        m_wheel_L.setPWM(4);
        break;

    // 前進
    case 8:
        lineRun(0, 20, 0, 35);
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
