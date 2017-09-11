// -*- C++ -*-
/*!
 * @file  CraneplusBridge.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <fstream>
#include "CraneplusBridge.h"
using namespace std;

template <typename T>
bool isfinite(T x)
{
  return (x == x) 
    && (x != std::numeric_limits<T>::infinity())
    && (x != -std::numeric_limits<T>::infinity());
}

int sign(double x);

const double OutputDt = 0.01; //[sec]
const double Dt = 0.002; //[sec]
const double SpeedToRps = 0.111*2*M_PI/60.0; //AX-12の内部表現値からrad/sへの変換係数
const double PositionToRad = 300.0/1024*M_PI/180.0; //AX-12の内部表現値からradへの変換係数
const double RadToPosition = 1/PositionToRad; //radからAX-12の内部表現値への変換係数

ofstream Ofs;

// Module specification
// <rtc-template block="module_spec">
static const char* craneplusbridge_spec[] =
  {
    "implementation_id", "CraneplusBridge",
    "type_name",         "CraneplusBridge",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "MasutaniLab",
    "category",          "Robot Arm",
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
CraneplusBridge::CraneplusBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_goalPositionIn("goalPosition", m_goalPosition),
    m_movingSpeedIn("movingSpeed", m_movingSpeed),
    m_armAngleIn("armAngle", m_armAngle),
    m_presentPositionOut("presentPosition", m_presentPosition),
    m_movingOut("moving", m_moving),
    m_armTorqueOut("armTorque", m_armTorque)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
CraneplusBridge::~CraneplusBridge()
{
}



RTC::ReturnCode_t CraneplusBridge::onInitialize()
{
  cout << "CraneplusBridge::onInitialize()" << endl;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("goalPosition", m_goalPositionIn);
  addInPort("movingSpeed", m_movingSpeedIn);
  addInPort("armAngle", m_armAngleIn);
  
  // Set OutPort buffer
  addOutPort("presentPosition", m_presentPositionOut);
  addOutPort("moving", m_movingOut);
  addOutPort("armTorque", m_armTorqueOut);
  
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
RTC::ReturnCode_t CraneplusBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CraneplusBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CraneplusBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t CraneplusBridge::onActivated(RTC::UniqueId ec_id)
{
  cout << "CraneplusBridge::onActivated()" << endl;

  m_armTotalTime.resize(m_NUM_ACTUATOR);
  m_armAccTime.resize(m_NUM_ACTUATOR);
  m_armAngleTarget.resize(m_NUM_ACTUATOR);
  m_armAnglePrev.resize(m_NUM_ACTUATOR);
  m_armVelocityLimit.resize(m_NUM_ACTUATOR);
  m_armAngleCurrent.resize(m_NUM_ACTUATOR);
  m_armVelocityTarget.resize(m_NUM_ACTUATOR);
  m_armStartAngle.resize(m_NUM_ACTUATOR);
  m_armSign.resize(m_NUM_ACTUATOR);
  m_armI.resize(m_NUM_ACTUATOR);
  for (int i=0; i<m_NUM_ACTUATOR; i++) {
    m_armAngleTarget[i] = 0;
    m_armAccTime[i] = 0;
    m_armTotalTime[i] = 0;
    m_armVelocityLimit[i] = 1023*SpeedToRps;
    m_armSign[i] = 1;
    m_armI[i] = 0;
  }
  m_armAngle.data.length(m_NUM_ACTUATOR);
  m_armTorque.data.length(m_NUM_ACTUATOR);
  m_moving.data.length(m_NUM_ACTUATOR);
  m_presentPosition.data.length(m_NUM_ACTUATOR);

  m_armFirst = true;
  m_armCurrentTime = 0;
  m_armStartTime = 0;
  m_armPrevOutputTime = 0;

  Ofs.open("CraneplusBridgeLog.txt");

  return RTC::RTC_OK;
}


RTC::ReturnCode_t CraneplusBridge::onDeactivated(RTC::UniqueId ec_id)
{
  cout << "CraneplusBridge::onDeactivated()" << endl;
  Ofs.close();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CraneplusBridge::onExecute(RTC::UniqueId ec_id)
{
  const double EpsAngle = 1e-3; // [rad]
  const double EpsVelocity = 1e-1; // [rad/s]
  const double AccelerationLimit = 20; // [rad/s^2]

  //ユーザRTC側から入力があった場合
  if (m_movingSpeedIn.isNew()) {
    m_movingSpeedIn.read();
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_armVelocityLimit[i] = m_movingSpeed.data[i]*SpeedToRps; 
    }
  }

  if (m_goalPositionIn.isNew()) {
    m_goalPositionIn.read();
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_armAngleTarget[i] = (m_goalPosition.data[i]-512)*PositionToRad;
    }
    m_armStartTime = m_armCurrentTime;
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_armStartAngle[i] = m_armAngleCurrent[i];
      double xt = m_armAngleTarget[i] - m_armStartAngle[i];
      m_armSign[i] = sign(xt);
      if (abs(xt) > pow(m_armVelocityLimit[i],2)/AccelerationLimit) {
        m_armAccTime[i] = m_armVelocityLimit[i]/AccelerationLimit;
        m_armTotalTime[i] = abs(xt)/m_armVelocityLimit[i] + m_armVelocityLimit[i]/AccelerationLimit;
      } else {
        m_armAccTime[i] = sqrt(abs(xt)/AccelerationLimit);
        m_armTotalTime[i] = 2*m_armAccTime[i];
      }
      cout << i << " " << m_armStartAngle[i] << " " << m_armTotalTime[i] << " " << m_armAccTime[i] << " " << m_armVelocityLimit[i] << endl;
    }
  }


  //BodyRTC側から入力があった場合
  if (m_armAngleIn.isNew()) {
    m_armAngleIn.read();
    m_armCurrentTime = m_armAngle.tm.sec + 1e-9*m_armAngle.tm.nsec;
    Ofs << "m_armCurrentTime: " << m_armCurrentTime << endl;
    if (m_armFirst || m_armPrevTime > m_armCurrentTime) {
      m_armPrevTime = m_armCurrentTime - Dt;
      m_armPrevOutputTime = m_armCurrentTime - OutputDt;
      for (int i=0; i<m_NUM_ACTUATOR; i++) {
        m_armAnglePrev[i] = m_armAngleCurrent[i];
      }
      m_armFirst = false;
    }
    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      m_armAngleCurrent[i] = m_armAngle.data[i];
    }
    double dt = m_armCurrentTime - m_armPrevTime;
    m_armPrevTime = m_armCurrentTime;

    for (int i=0; i<m_NUM_ACTUATOR; i++) {
      //以前の角度と現在角度からアームが動作中か停止中かを判断
      if (abs(m_armAngleCurrent[i]-m_armAnglePrev[i]) > EpsAngle) {
        m_moving.data[i] = 1;
      } else {
        m_moving.data[i] = 0;
      }

      //各軸ごとのPID制御
      double xd; //現在の目標角度
      double vd; //現在の目標速度
      double t = m_armCurrentTime - m_armStartTime;
      if (t < m_armAccTime[i]) { //加速区間
        xd = m_armStartAngle[i] + m_armSign[i]*AccelerationLimit*pow(t,2)/2;
        vd = m_armSign[i]*AccelerationLimit*t;
      } else if (t < m_armTotalTime[i]-m_armAccTime[i]) { //等速区間
        xd = m_armStartAngle[i] + m_armSign[i]*(AccelerationLimit*pow(m_armAccTime[i],2)/2+m_armVelocityLimit[i]*(t-m_armAccTime[i]));
        vd = m_armSign[i]*m_armVelocityLimit[i];
      } else if (t < m_armTotalTime[i]) {//減速区間
        xd = m_armAngleTarget[i] - m_armSign[i]*AccelerationLimit*pow(m_armTotalTime[i]-t,2)/2;
        vd = m_armSign[i]*AccelerationLimit*(m_armTotalTime[i]-t);
      } else { //設定時間経過後
        //cout << i << " ";
        xd = m_armAngleTarget[i];
        vd = 0;
      }
      double v = (m_armAngleCurrent[i] - m_armAnglePrev[i])/dt;
      double dx = m_armAngleCurrent[i] - xd;
      double dv = v - vd;
      if (abs(m_armI[i]) < 10) {
        m_armI[i] += dx*dt;
      }
      const double kp = 10;
      const double kd = 1;
      const double ki = 2;
      double torque = -kp*dx -kd*dv -ki*m_armI[i];
      if (!isfinite(torque)) {
        torque = 0;
      }
      m_armTorque.data[i] = torque;
      m_armAnglePrev[i] = m_armAngleCurrent[i];//一つ前の関節角度を保持
    }
    //for (int i=0; i<m_NUM_ACTUATOR; i++) {
    //  cout << i << " " << m_armTorque.data[i] << endl;
    //}
    //cout << endl;
    m_armTorqueOut.write();

    if (m_armCurrentTime-m_armPrevOutputTime > OutputDt-0.9*Dt) {
      m_moving.tm = m_armAngle.tm;
      m_movingOut.write();
      m_presentPosition.tm = m_armAngle.tm;
      for (int i=0; i<m_NUM_ACTUATOR; i++) {
        m_presentPosition.data[i] = m_armAngleCurrent[i]*RadToPosition + 512;
      }
      m_presentPositionOut.write();
      m_armPrevOutputTime = m_armCurrentTime;
    }
  }

  return RTC::RTC_OK;
}


RTC::ReturnCode_t CraneplusBridge::onAborting(RTC::UniqueId ec_id)
{
  cout << "CraneplusBridge::onAborting()" << endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CraneplusBridge::onError(RTC::UniqueId ec_id)
{
  //cout << "CraneplusBridge::onError()" << endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CraneplusBridge::onReset(RTC::UniqueId ec_id)
{
  cout << "CraneplusBridge::onReset()" << endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CraneplusBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CraneplusBridge::onRateChanged(RTC::UniqueId ec_id)
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
 
  void CraneplusBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(craneplusbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<CraneplusBridge>,
                             RTC::Delete<CraneplusBridge>);
  }
  
};


