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

    // ライントレース
    case 1:
        lineRun(1, 80, 0, 35);
        if (seesaw_dis > 0.40)
        {
            ev3_led_set_color(LED_ORANGE);
            ev3_speaker_play_tone(262, 100);
            start_time = m_clock.now();
            m_sequence_num++;
        }
        break;

    // 着地と姿勢上げ
    case 2:
        // 着地
        m_tail->setAngle(70);
        m_tail->setMaxSpeed(40);
        m_wheel_L.setPWM(90);
        m_wheel_R.setPWM(90);
        m_clock.sleep(160);

        m_guage->setOdomOffset(0);
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(3000);

        // 姿勢上げ
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(55);
        m_wheel_L.setPWM(-45);
        m_wheel_R.setPWM(-45);
        m_clock.sleep(120);

        m_tail->setAngle(97);
        m_tail->setMaxSpeed(90);
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(1000);

        m_sequence_num++;
        break;

    // 前進して車輪を乗り上げる
    case 3:
        // 前進
        lineRun(0, 6, 1, 15);

        // 車輪を乗り上げ後
        if (m_guage->getRobotDis() > 0.30)
        {
            ev3_speaker_play_tone(262, 100);

            // 左右にじたばた
            for (int i = 0; i < 3; i++)
            {
                m_wheel_L.setPWM(8);
                m_wheel_R.reset();
                m_clock.sleep(1500);
                m_wheel_L.reset();
                m_wheel_R.setPWM(8);
                m_clock.sleep(1500);
            }

            m_tail->setAngle(1000);
            m_tail->setMaxSpeed(60);
            m_wheel_L.setPWM(-55);
            m_wheel_R.setPWM(-55);
            m_clock.sleep(220);

            m_tail->setAngle(120);
            m_tail->setMaxSpeed(90);
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);

            // 左右にじたばた
            for (int i = 0; i < 3; i++)
            {
                m_wheel_L.setPWM(8);
                m_wheel_R.reset();
                m_clock.sleep(1500);
                m_wheel_L.reset();
                m_wheel_R.setPWM(8);
                m_clock.sleep(1500);
            }

            // 前進
            m_wheel_L.setPWM(10);
            m_wheel_R.setPWM(10);
            m_clock.sleep(1000);

            // 停止
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);

            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    // 昇段
    case 4:
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(20);
        m_wheel_L.setPWM(-8);
        m_wheel_R.setPWM(-8);
        if (m_guage->getWheelPos(GuageManager::L) < -22)
        {
            m_tail->setAngle(105);
            m_tail->setMaxSpeed(25);
            m_sequence_num++;
        }
        break;

    // 昇段
    case 5:
        m_tail->setMaxSpeed(90);
        m_wheel_L.setPWM(100);
        m_wheel_R.setPWM(100);
        if (m_guage->getWheelPos(GuageManager::L) > 180)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 6:
        lineRun(0, 4, 1, 25);
        if (m_guage->getRobotDis() > 0.14)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_sequence_num++;
        }
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
