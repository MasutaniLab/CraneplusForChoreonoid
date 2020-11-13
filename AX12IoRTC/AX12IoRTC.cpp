/**
   @author Yasuhiro Masutani
*/

#define _USE_MATH_DEFINES
#include <cnoid/BodyIoRTC>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cmath>
#include <vector>

using namespace std;
using namespace cnoid;

int sign(double x);

namespace {

class AX12IoRTC : public BodyIoRTC
{
public:
    AX12IoRTC(RTC::Manager* manager);
    ~AX12IoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;
    virtual void outputToSimulator() override;
    
    // Configuration variable declaration
    short int m_NUM_ACTUATOR;

    // DataInPort declaration
    RTC::TimedUShortSeq m_goalPosition;
    RTC::InPort<RTC::TimedUShortSeq> m_goalPositionIn;

    RTC::TimedUShortSeq m_movingSpeed;
    RTC::InPort<RTC::TimedUShortSeq> m_movingSpeedIn;

    // DataOutPort declaration
    RTC::TimedUShortSeq m_presentPosition;
    RTC::OutPort<RTC::TimedUShortSeq> m_presentPositionOut;

    RTC::TimedUShortSeq m_moving;
    RTC::OutPort<RTC::TimedUShortSeq> m_movingOut;

private:
  ControllerIO* m_io;
  BodyPtr m_ioBody;
  std::vector<Link*>  m_links;

  double m_prevTime;
  double m_currentTime;
  double m_prevOutputTime;
  double m_startTime;
  std::vector<double> m_totalTime; //全体時間区間
  std::vector<double> m_accTime; //加速時間区間
  std::vector<double> m_angleTarget; //関節角度指令値
  std::vector<double> m_anglePrev; //一つ前の関節角度
  std::vector<double> m_velocityLimit; //関節速度上限
  std::vector<double> m_angleCurrent; //現在の関節角度
  std::vector<double> m_angleStart; //制御開始関節角度
  std::vector<double> m_sign; //動作方向
  std::vector<double> m_integral; //偏差の積分
  bool m_first;
};

#if _MSC_VER <= 1900
template <typename T>
bool isfinite(T x)
{
  return (x == x)
    && (x != std::numeric_limits<T>::infinity())
    && (x != -std::numeric_limits<T>::infinity());
}
#endif


const double OutputDt = 0.01; //[sec]
const double Dt = 0.002; //[sec]
const double SpeedToRps = 0.111 * 2 * M_PI / 60.0; //AX-12の内部表現値からrad/sへの変換係数
const double PositionToRad = 300.0 / 1024 * M_PI / 180.0; //AX-12の内部表現値からradへの変換係数
const double RadToPosition = 1 / PositionToRad; //radからAX-12の内部表現値への変換係数
const double MaxVelocity = 59 * 2 * M_PI / 60.0; //AX-12の仕様 無負荷速度

const double EpsAngle = 2e-2; // [rad]
const double EpsVelocity = 1e-1; // [rad/s]
const double AccelerationLimit = 20; // [rad/s^2]

const char* spec[] =
{
    "implementation_id", "AX12IoRTC",
    "type_name",         "AX12IoRTC",
    "description",       "Dynamixel AX12 I/O",
    "version",           "1.0",
    "vendor",            "MasutaniLab",
    "category",          "Actuator",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.NUM_ACTUATOR", "5",
    // Widget
    "conf.__widget__.NUM_ACTUATOR", "text",
    // Constraints
    "conf.__type__.NUM_ACTUATOR", "short",
    ""
};

}

AX12IoRTC::AX12IoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      m_goalPositionIn("goalPosition", m_goalPosition),
      m_movingSpeedIn("movingSpeed", m_movingSpeed),
      m_presentPositionOut("presentPosition", m_presentPosition),
      m_movingOut("moving", m_moving)
{

}


AX12IoRTC::~AX12IoRTC()
{

}


bool AX12IoRTC::initializeIO(ControllerIO* io)
{
  // Set InPort buffers
  addInPort("goalPosition", m_goalPositionIn);
  addInPort("movingSpeed", m_movingSpeedIn);

  // Set OutPort buffer
  addOutPort("presentPosition", m_presentPositionOut);
  addOutPort("moving", m_movingOut);

  // Bind variables and configuration variable
  bindParameter("NUM_ACTUATOR", m_NUM_ACTUATOR, "5");

  return true;
}


bool AX12IoRTC::initializeSimulation(ControllerIO* io)
{
  m_io = io;
  m_ioBody = io->body();
  m_links.reserve(m_NUM_ACTUATOR);
  //リンクの名前は J1, J2, J3, ...と仮定
  for (int i = 0; i < m_NUM_ACTUATOR; i++) {
    m_links[i] = m_ioBody->link("J" + to_string(i+1));
    m_links[i]->setActuationMode(Link::JOINT_TORQUE);
  }

  m_totalTime.resize(m_NUM_ACTUATOR);
  m_accTime.resize(m_NUM_ACTUATOR);
  m_angleTarget.resize(m_NUM_ACTUATOR);
  m_anglePrev.resize(m_NUM_ACTUATOR);
  m_velocityLimit.resize(m_NUM_ACTUATOR);
  m_angleCurrent.resize(m_NUM_ACTUATOR);
  m_angleStart.resize(m_NUM_ACTUATOR);
  m_sign.resize(m_NUM_ACTUATOR);
  m_integral.resize(m_NUM_ACTUATOR);
  for (int i = 0; i < m_NUM_ACTUATOR; i++) {
    m_angleTarget[i] = 0;
    m_accTime[i] = 0;
    m_totalTime[i] = 0;
    m_velocityLimit[i] = 1023 * SpeedToRps;
    m_sign[i] = 1;
    m_integral[i] = 0;
  }
  m_moving.data.length(m_NUM_ACTUATOR);
  m_presentPosition.data.length(m_NUM_ACTUATOR);

  m_first = true;
  m_currentTime = 0;
  m_startTime = 0;
  m_prevOutputTime = 0;

  return true;
}

void AX12IoRTC::inputFromSimulator()
{
  m_currentTime = m_io->currentTime(); //要確認
  if (m_first || m_prevTime > m_currentTime) {
    m_prevTime = m_currentTime - Dt;
    m_prevOutputTime = m_currentTime - OutputDt;
    for (int i = 0; i < m_NUM_ACTUATOR; i++) {
      m_anglePrev[i] = m_angleCurrent[i];
    }
    m_first = false;
  }
  for (int i = 0; i < m_NUM_ACTUATOR; i++) {
    m_angleCurrent[i] = m_links[i]->q();
  }
  double dt = m_currentTime - m_prevTime;
  m_prevTime = m_currentTime;

  for (int i = 0; i < m_NUM_ACTUATOR; i++) {
    //以前の角度と現在角度からアームが動作中か停止中かを判断
    if (abs(m_angleCurrent[i] - m_anglePrev[i]) > EpsAngle) {
      m_moving.data[i] = 1;
    } else {
      m_moving.data[i] = 0;
    }

    //各軸ごとのPID制御
    double xd; //現在の目標角度
    double vd; //現在の目標速度
    double t = m_currentTime - m_startTime;
    if (t < m_accTime[i]) { //加速区間
      xd = m_angleStart[i] + m_sign[i] * AccelerationLimit * pow(t, 2) / 2;
      vd = m_sign[i] * AccelerationLimit * t;
    } else if (t < m_totalTime[i] - m_accTime[i]) { //等速区間
      xd = m_angleStart[i] + m_sign[i] * (AccelerationLimit * pow(m_accTime[i], 2) / 2 + m_velocityLimit[i] * (t - m_accTime[i]));
      vd = m_sign[i] * m_velocityLimit[i];
    } else if (t < m_totalTime[i]) {//減速区間
      xd = m_angleTarget[i] - m_sign[i] * AccelerationLimit * pow(m_totalTime[i] - t, 2) / 2;
      vd = m_sign[i] * AccelerationLimit * (m_totalTime[i] - t);
    } else { //設定時間経過後
      //cout << i << " ";
      xd = m_angleTarget[i];
      vd = 0;
    }
    double v = (m_angleCurrent[i] - m_anglePrev[i]) / dt;
    double dx = m_angleCurrent[i] - xd;
    double dv = v - vd;
    if (abs(m_integral[i]) < 10) { //要検討：積分値の上限
      m_integral[i] += dx * dt;
    }

    //要検討：PIDフィードバックゲイン
    const double kp = 39.5;
    const double kd = 1.5;
    const double ki = 10;

    double torque = -kp * dx - kd * dv - ki * m_integral[i];

    //トルクが異常な値になった時の予防
    if (!isfinite(torque)) {
      torque = 0;
    }
    //トルクの上限を超えないように
    const double maxTorque = 1.5; //[Nm] AX12の仕様上の停動トルク
    if (torque > maxTorque) {
      torque = maxTorque;
    } else if (torque < -maxTorque) {
      torque = -maxTorque;
    }
    m_links[i]->u() = torque;
    m_anglePrev[i] = m_angleCurrent[i];//一つ前の関節角度を保持
  }

  if (m_currentTime - m_prevOutputTime > OutputDt - 0.9 * Dt) {
    RTC::Time tm;
    coil::TimeValue ct(m_currentTime);
    tm.sec = ct.sec();
    tm.nsec = ct.usec() * 1000;
    m_moving.tm = tm;
    m_movingOut.write();
    m_presentPosition.tm = tm;
    for (int i = 0; i < m_NUM_ACTUATOR; i++) {
      m_presentPosition.data[i] = m_angleCurrent[i] * RadToPosition + 512;
    }
    m_presentPositionOut.write();
    m_prevOutputTime = m_currentTime;
  }
}

RTC::ReturnCode_t AX12IoRTC::onExecute(RTC::UniqueId ec_id)
{
  if (m_movingSpeedIn.isNew()) {
    m_movingSpeedIn.read();
    for (int i = 0; i < m_NUM_ACTUATOR; i++) {
      if (m_movingSpeed.data[i] == 0) {
        m_velocityLimit[i] = MaxVelocity;
      } else {
        m_velocityLimit[i] = m_movingSpeed.data[i] * SpeedToRps;
        if (m_velocityLimit[i] > MaxVelocity) {
          m_velocityLimit[i] = MaxVelocity;
        }
      }
    }
  }

  if (m_goalPositionIn.isNew()) {
    m_goalPositionIn.read();
    for (int i = 0; i < m_NUM_ACTUATOR; i++) {
      m_angleTarget[i] = (m_goalPosition.data[i] - 512) * PositionToRad;
    }
    m_startTime = m_currentTime;
    for (int i = 0; i < m_NUM_ACTUATOR; i++) {
      m_angleStart[i] = m_angleCurrent[i];
      double xt = m_angleTarget[i] - m_angleStart[i];
      m_sign[i] = sign(xt);
      if (abs(xt) > pow(m_velocityLimit[i], 2) / AccelerationLimit) {
        m_accTime[i] = m_velocityLimit[i] / AccelerationLimit;
        m_totalTime[i] = abs(xt) / m_velocityLimit[i] + m_velocityLimit[i] / AccelerationLimit;
      } else {
        m_accTime[i] = sqrt(abs(xt) / AccelerationLimit);
        m_totalTime[i] = 2 * m_accTime[i];
      }
      cout << i << " " << m_angleStart[i] << " " << m_totalTime[i] << " " << m_accTime[i] << " " << m_velocityLimit[i] << endl;
    }
  }

  return RTC::RTC_OK;
}

void AX12IoRTC::outputToSimulator()
{
}


//引数の符号を返す
int
sign(double x)
{
  if (x < 0) return -1;
  if (x > 0) return 1;
  return 0;
}

extern "C"
{
    DLL_EXPORT void AX12IoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<AX12IoRTC>, RTC::Delete<AX12IoRTC>);
    }
};
