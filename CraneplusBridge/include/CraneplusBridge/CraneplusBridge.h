// -*- C++ -*-
/*!
 * @file  CraneplusBridge.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef CRANEPLUSBRIDGE_H
#define CRANEPLUSBRIDGE_H

#include <vector>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

/*!
 * @class CraneplusBridge
 * @brief ${rtcParam.description}
 *
 */
class CraneplusBridge
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  CraneplusBridge(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~CraneplusBridge();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  NUM_ACTUATOR
   * - DefaultValue: 5
   */
  short int m_NUM_ACTUATOR;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedUShortSeq m_goalPosition;
  /*!
   */
  InPort<RTC::TimedUShortSeq> m_goalPositionIn;
  RTC::TimedUShortSeq m_movingSpeed;
  /*!
   */
  InPort<RTC::TimedUShortSeq> m_movingSpeedIn;
  RTC::TimedDoubleSeq m_armAngle;
  /*!
   */
  InPort<RTC::TimedDoubleSeq> m_armAngleIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedUShortSeq m_presentPosition;
  /*!
   */
  OutPort<RTC::TimedUShortSeq> m_presentPositionOut;
  RTC::TimedUShortSeq m_moving;
  /*!
   */
  OutPort<RTC::TimedUShortSeq> m_movingOut;
  RTC::TimedDoubleSeq m_armTorque;
  /*!
   */
  OutPort<RTC::TimedDoubleSeq> m_armTorqueOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>
   double m_armPrevTime;
   double m_armCurrentTime;
   double m_armPrevOutputTime;
   double m_armStartTime;
   std::vector<double> m_armTotalTime; //全体時間区間
   std::vector<double> m_armAccTime; //加速時間区間
   std::vector<double> m_armAngleTarget; //関節角度指令値（関節1～5 0は使わない）
   std::vector<double> m_armAnglePrev; //一つ前の関節角度（関節1～5 0は使わない）
   std::vector<double> m_armVelocityLimit; //関節速度上限（関節1～5 0は使わない）
   std::vector<double> m_armAngleCurrent; //現在の関節角度（関節1～5 0は使わない）
   std::vector<double> m_armVelocityTarget; //関節速度指令値（関節1～5 0は使わない）
   std::vector<double> m_armStartAngle; //制御開始関節角度（関節1～5 0は使わない）
   std::vector<double> m_armSign; //動作方向（関節1～5 0は使わない）
   std::vector<double> m_armI; //偏差の積分（関節1～5 0は使わない）
   bool m_armFirst;

};


extern "C"
{
  DLL_EXPORT void CraneplusBridgeInit(RTC::Manager* manager);
};

#endif // CRANEPLUSBRIDGE_H
