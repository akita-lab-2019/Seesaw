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
        // m_sequence_num++;
        m_sequence_num = 3;

        m_wheel_L.reset();
        m_wheel_R.reset();
        m_tail->setAngle(55);
        m_clock.sleep(3000);
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
        m_tail->setMaxSpeed(50);
        m_wheel_L.setPWM(-35);
        m_wheel_R.setPWM(-35);
        m_clock.sleep(100);

        m_tail->setAngle(95);
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

            // // 左右にじたばた
            // for (int i = 0; i < 3; i++)
            // {
            //     m_wheel_L.setPWM(8);
            //     m_wheel_R.reset();
            //     m_clock.sleep(800);
            //     m_wheel_L.reset();
            //     m_wheel_R.setPWM(8);
            //     m_clock.sleep(800);
            // }

            // m_tail->setAngle(1000);
            // m_tail->setMaxSpeed(60);
            // m_wheel_L.setPWM(-55);
            // m_wheel_R.setPWM(-55);
            // m_clock.sleep(220);

            // m_tail->setAngle(115);
            // m_tail->setMaxSpeed(90);
            // m_wheel_L.reset();
            // m_wheel_R.reset();
            // m_clock.sleep(1000);

            // // 左右にじたばた
            // for (int i = 0; i < 3; i++)
            // {
            //     m_wheel_L.setPWM(8);
            //     m_wheel_R.reset();
            //     m_clock.sleep(800);
            //     m_wheel_L.reset();
            //     m_wheel_R.setPWM(8);
            //     m_clock.sleep(800);
            // }

            m_wheel_L.reset();
            m_wheel_R.reset();
            m_tail->setAngle(115);
            m_clock.sleep(3000);

            // 前進
            m_tail->setAngle(115);
            m_tail->setMaxSpeed(100);
            m_wheel_L.setPWM(10);
            m_wheel_R.setPWM(10);
            m_clock.sleep(1000);

            // 停止
            m_guage->setOdomOffset(0);
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);

            ev3_speaker_play_tone(262, 100);
            m_sequence_num++;
        }
        break;

    // 昇段
    case 4:
        m_wheel_L.setPWM(-10);
        m_wheel_R.setPWM(-10);

        if (m_guage->getWheelPos(GuageManager::L) < -10)
        {
            m_sequence_num++;
        }
        break;

    // 昇段
    case 5:
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(35);
        m_wheel_L.setPWM(-11);
        m_wheel_R.setPWM(-11);

        if (m_guage->getWheelPos(GuageManager::L) < -20)
        {
            m_sequence_num++;
        }
        break;

    // 昇段
    case 6:
        m_tail->setAngle(-1000);
        m_tail->setMaxSpeed(100);
        m_wheel_L.setPWM(-11);
        m_wheel_R.setPWM(-11);

        if (m_guage->getWheelPos(GuageManager::L) < -32)
        {
            m_sequence_num++;
        }
        break;

    // 昇段
    case 7:
        m_wheel_L.setPWM(95);
        m_wheel_R.setPWM(95);
        m_tail->setAngle(92);
        m_tail->setMaxSpeed(100);

        if (m_guage->getRobotDis() > 0.15)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);

            m_tail->setAngle(1000);
            m_tail->setMaxSpeed(50);
            m_wheel_L.setPWM(-33);
            m_wheel_R.setPWM(-33);
            m_clock.sleep(130);

            m_tail->setAngle(108);
            m_tail->setMaxSpeed(90);
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 8:
        lineRun(0, 4, 1, 25);
        if (m_guage->getPitchVel() > 40)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_sequence_num++;
        }
        break;

    // 前進
    case 9:
        m_tail->setAngle(-1000);
        m_tail->setMaxSpeed(55);
        m_wheel_L.setPWM(35);
        m_wheel_R.setPWM(35);
        if (m_guage->getRobotDis() > 0.03)
        {
            m_sequence_num++;
        }
        break;

    // 前進
    case 10:
        m_tail->setAngle(52);
        m_tail->setMaxSpeed(55);
        m_wheel_L.setPWM(30);
        m_wheel_R.setPWM(30);
        if (m_guage->getRobotDis() > 0.10)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 11:
        m_tail->setAngle(52);
        m_tail->setMaxSpeed(60);
        m_wheel_L.setPWM(-3);
        m_wheel_R.setPWM(-3);

        if (m_guage->getPitchVel() < -30)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_sequence_num++;
        }
        break;

    // 前進
    case 12:
        m_tail->setAngle(1000);
        m_tail->setMaxSpeed(100);
        m_wheel_L.setPWM(-30);
        m_wheel_R.setPWM(-30);
        m_clock.sleep(200);

        m_tail->setAngle(108);
        m_tail->setMaxSpeed(90);
        m_wheel_L.reset();
        m_wheel_R.reset();
        m_clock.sleep(3000);
        m_sequence_num++;
        break;

    // 前進
    case 13:
        m_wheel_L.setPWM(-5);
        m_wheel_R.setPWM(-5);
        if (m_guage->getRobotDis() < -0.10)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 14:
        lineRun(0, 4, 1, 25);
        if (m_guage->getPitchVel() > 40)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_sequence_num++;
        }
        break;

    // 前進
    case 15:
        m_tail->setAngle(-1000);
        m_tail->setMaxSpeed(55);
        m_wheel_L.setPWM(50);
        m_wheel_R.setPWM(50);
        if (m_guage->getRobotDis() > 0.03)
        {
            m_sequence_num++;
        }
        break;

    // 前進
    case 16:
        m_tail->setAngle(50);
        m_tail->setMaxSpeed(55);
        m_wheel_L.setPWM(40);
        m_wheel_R.setPWM(40);
        if (m_guage->getRobotDis() > 0.13)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);
            m_sequence_num++;
        }
        break;

    // 前進
    case 17:
        m_tail->setAngle(55);
        m_tail->setMaxSpeed(100);
        m_wheel_L.setPWM(30);
        m_wheel_R.setPWM(30);
        if (m_guage->getRobotDis() > 0.35)
        {
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(3000);
            m_tail->setAngle(1000);
            m_tail->setMaxSpeed(100);
            m_wheel_L.setPWM(-70);
            m_wheel_R.setPWM(-70);
            m_clock.sleep(210);

            m_tail->setAngle(95);
            m_tail->setMaxSpeed(90);
            m_wheel_L.reset();
            m_wheel_R.reset();
            m_clock.sleep(1000);
            m_wheel_L.setPWM(-5);
            m_wheel_R.setPWM(5);
            m_clock.sleep(500);
            m_sequence_num++;
        }
        break;

    // // 前進
    case 18:
        lineRun(0, 8, 1, 15);

        if (m_guage->getRobotDis() > 0.35)
        {
            ev3_speaker_play_tone(262, 1000);
            m_sequence_num++;
        }
        break;

    // 停止
    case 19:
        m_wheel_L.reset();
        m_wheel_R.reset();
        break;

    // 前進
    default:
        m_wheel_L.reset();
        m_wheel_R.reset();
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
    m_tail->setAngle(40);
    m_tail->setMaxSpeed(100);
}

/**
 * ライントレースする
 *
 */
void Seesaw::lineRun(bool is_inverted, int forward, int pid_index, int target)
{
    m_line_tracer->setTurnMax(2);
    m_line_tracer->setIsInverted(is_inverted);
    m_line_tracer->setForward(forward);
    m_line_tracer->setCurvature(0);
    m_line_tracer->setColorTarget(target);
    m_line_tracer->run();
}

void Seesaw::landing()
{
}
