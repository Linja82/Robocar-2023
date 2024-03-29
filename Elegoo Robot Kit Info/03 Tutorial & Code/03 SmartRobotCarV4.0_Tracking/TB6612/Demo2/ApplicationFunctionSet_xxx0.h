/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 15:46:13
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Tracking(void);           //循迹
  void ApplicationFunctionSet_SensorDataUpdate(void);   //传感器数据更新
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  
private:
  volatile float TrackingData_L;       //循迹数据
  volatile float TrackingData_M;       //循迹数据
  volatile float TrackingData_R;

  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;

public:
  uint16_t TrackingDetection_S = 40;
  uint16_t TrackingDetection_E = 400;
  uint16_t TrackingDetection_V = 950;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
