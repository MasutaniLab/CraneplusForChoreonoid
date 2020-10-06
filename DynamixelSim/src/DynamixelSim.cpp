// -*- C++ -*-
/*!
 * @file  DynamixelSim.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <fstream>
#include "DynamixelSim.h"
using namespace std;

#if _MSC_VER <= 1900
template <typename T>
bool isfinite(T x)
{
  return (x == x) 
    && (x != std::numeric_limits<T>::infinity())
    && (x != -std::numeric_limits<T>::infinity());
}
#endif

int sign(double x);

const double OutputDt = 0.01; //[sec]
const double Dt = 0.002; //[sec]
const double SpeedToRps = 0.111*2*M_PI/60.0; //AX-12の内部表現値からrad/sへの変換係数
const double PositionToRad = 300.0/1024*M_PI/180.0; //AX-12の内部表現値からradへの変換係数
const double RadToPosition = 1/PositionToRad; //radからAX-12の内部表現値への変換係数
const double MaxVelocity = 59*2*M_PI/60.0; //AX-12の仕様 無負荷速度

// Module specification
// <rtc-template block="module_spec">
static const char* dynamixelsim_spec[] =
  {
    "implementation_id", "DynamixelSim",
    "type_name",         "DynamixelSim",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "MasutaniLab",
    "category",          "Actuator",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
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
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
DynamixelSim::DynamixelSim(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_goalPositionIn("goalPosition", m_goalPosition),
    m_movingSpeedIn("movingSpeed", m_movingSpeed),
    m_angleIn("angle", m_angle),
    m_presentPositionOut("presentPosition", m_presentPosition),
    m_movingOut("moving", m_moving),
    m_torqueOut("torque", m_torque)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
DynamixelSim::~DynamixelSim()
{
}



RTC::ReturnCode_t DynamixelSim::onInitialize()
{
  cout << "DynamixelSim::onInitialize()" << endl;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("goalPosition", m_goalPositionIn);
  addInPort("movingSpeed", m_movingSpeedIn);
  addInPort("angle", m_angleIn);
  
  // Set OutPort buffer
  addOutPort("presentPosition", m_presentPositionOut);
  addOutPort("moving", m_movingOut);
  addOutPort("torque", m_torqueOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("NUM_ACTUATOR", m_NUM_ACTUATOR, "5");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DynamixelSim::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DynamixelSim::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DynamixelSim::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t DynamixelSim::onActivated(RTC::UniqueId ec_id)
{
  cout << "DynamixelSim::onActivated()" << endl;

  m_totalTime.resize(m_NUM_ACTUATOR);
  m_accTime.resize(m_NUM_ACTUATOR);
  m_angleTarget.resize(m_NUM_ACTUATOR);
  m_anglePrev.resize(m_NUM_ACTUATOR);
  m_velocityLimit.resize(m_NUM_ACTUATOR);
  m_angleCurrent.resize(m_NUM_ACTUATOR);
  m_angleStart.resize(m_NUM_ACTUATOR);
  m_sign.resize(m_NUM_ACTUATOR);
  m_integral.resize(m_NUM_ACTUATOR);
  for (int i=0; i<m_NUM_ACTUATOR; i++) {
    m_angleTarget[i] = 0;
    m_accTime[i] = 0;
    m_totalTime[i] = 0;
    m_velocityLimit[i] = 1023*SpeedToRps;
    m_sign[i] = 1;
    m_integral[i] = 0;
  }
  m_angle.data.length(m_NUM_ACTUATOR);
  m_torque.data.length(m_NUM_ACTUATOR);
  m_moving.data.length(m_NUM_ACTUATOR);
  m_presentPosition.data.length(m_NUM_ACTUATOR);

  m_first = true;
  m_currentTime = 0;
  m_startTime = 0;
  m_prevOutputTime = 0;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t DynamixelSim::onDeactivated(RTC::UniqueId ec_id)
{
  cout << "DynamixelSim::onDeactivated()" << endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DynamixelSim::onExecute(RTC::UniqueId ec_id)
{
  const double EpsAngle = 2e-2; // [rad]
  const double EpsVelocity = 1e-1; // [rad/s]
  const double AccelerationLimit = 20; // [rad/s^2]

  //ユーザRTC側から入力があった場合
  if (m_movingSpeedIn.isNew()) {
    m_movingSpeedIn.read();
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      if (m_movingSpeed.data[i] == 0) {
        m_velocityLimit[i] = MaxVelocity;
      } else {
        m_velocityLimit[i] = m_movingSpeed.data[i]*SpeedToRps;
        if (m_velocityLimit[i] > MaxVelocity) {
          m_velocityLimit[i] = MaxVelocity;
        }
      }
    }
  }

  if (m_goalPositionIn.isNew()) {
    m_goalPositionIn.read();
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_angleTarget[i] = (m_goalPosition.data[i]-512)*PositionToRad;
    }
    m_startTime = m_currentTime;
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_angleStart[i] = m_angleCurrent[i];
      double xt = m_angleTarget[i] - m_angleStart[i];
      m_sign[i] = sign(xt);
      if (abs(xt) > pow(m_velocityLimit[i],2)/AccelerationLimit) {
        m_accTime[i] = m_velocityLimit[i]/AccelerationLimit;
        m_totalTime[i] = abs(xt)/m_velocityLimit[i] + m_velocityLimit[i]/AccelerationLimit;
      } else {
        m_accTime[i] = sqrt(abs(xt)/AccelerationLimit);
        m_totalTime[i] = 2*m_accTime[i];
      }
      cout << i << " " << m_angleStart[i] << " " << m_totalTime[i] << " " << m_accTime[i] << " " << m_velocityLimit[i] << endl;
    }
  }


  //BodyRTC側から入力があった場合
  if (m_angleIn.isNew()) {
    m_angleIn.read();
    m_currentTime = m_angle.tm.sec + 1e-9*m_angle.tm.nsec;
    if (m_first || m_prevTime > m_currentTime) {
      m_prevTime = m_currentTime - Dt;
      m_prevOutputTime = m_currentTime - OutputDt;
      for (int i=0; i<m_NUM_ACTUATOR; i++) {
        m_anglePrev[i] = m_angleCurrent[i];
      }
      m_first = false;
    }
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_angleCurrent[i] = m_angle.data[i];
    }
    double dt = m_currentTime - m_prevTime;
    m_prevTime = m_currentTime;

    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      //以前の角度と現在角度からアームが動作中か停止中かを判断
      if (abs(m_angleCurrent[i]-m_anglePrev[i]) > EpsAngle) {
        m_moving.data[i] = 1;
      } else {
        m_moving.data[i] = 0;
      }

      //各軸ごとのPID制御
      double xd; //現在の目標角度
      double vd; //現在の目標速度
      double t = m_currentTime - m_startTime;
      if (t < m_accTime[i]) { //加速区間
        xd = m_angleStart[i] + m_sign[i]*AccelerationLimit*pow(t,2)/2;
        vd = m_sign[i]*AccelerationLimit*t;
      } else if (t < m_totalTime[i]-m_accTime[i]) { //等速区間
        xd = m_angleStart[i] + m_sign[i]*(AccelerationLimit*pow(m_accTime[i],2)/2+m_velocityLimit[i]*(t-m_accTime[i]));
        vd = m_sign[i]*m_velocityLimit[i];
      } else if (t < m_totalTime[i]) {//減速区間
        xd = m_angleTarget[i] - m_sign[i]*AccelerationLimit*pow(m_totalTime[i]-t,2)/2;
        vd = m_sign[i]*AccelerationLimit*(m_totalTime[i]-t);
      } else { //設定時間経過後
        //cout << i << " ";
        xd = m_angleTarget[i];
        vd = 0;
      }
      double v = (m_angleCurrent[i] - m_anglePrev[i])/dt;
      double dx = m_angleCurrent[i] - xd;
      double dv = v - vd;
      if (abs(m_integral[i]) < 10) { //要検討：積分値の上限
        m_integral[i] += dx*dt;
      }

      //要検討：PIDフィードバックゲイン
      const double kp = 39.5;
      const double kd = 1.5;
      const double ki = 10;

      double torque = -kp*dx -kd*dv -ki*m_integral[i];

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
      m_torque.data[i] = torque;
      m_anglePrev[i] = m_angleCurrent[i];//一つ前の関節角度を保持
    }
    m_torqueOut.write();

    if (m_currentTime-m_prevOutputTime > OutputDt-0.9*Dt) {
      m_moving.tm = m_angle.tm;
      m_movingOut.write();
      m_presentPosition.tm = m_angle.tm;
      for (int i=0; i<m_NUM_ACTUATOR; i++) {
        m_presentPosition.data[i] = m_angleCurrent[i]*RadToPosition + 512;
      }
      m_presentPositionOut.write();
      m_prevOutputTime = m_currentTime;
    }
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t DynamixelSim::onAborting(RTC::UniqueId ec_id)
{
  cout << "DynamixelSim::onAborting()" << endl;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t DynamixelSim::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DynamixelSim::onReset(RTC::UniqueId ec_id)
{
  cout << "DynamixelSim::onReset()" << endl;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DynamixelSim::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DynamixelSim::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

//引数の符号を返す
int
  sign(double x)
{
  if (x<0) return -1;
  if (x>0) return 1;
  return 0;
}


extern "C"
{
 
  void DynamixelSimInit(RTC::Manager* manager)
  {
    coil::Properties profile(dynamixelsim_spec);
    manager->registerFactory(profile,
                             RTC::Create<DynamixelSim>,
                             RTC::Delete<DynamixelSim>);
  }
  
};


