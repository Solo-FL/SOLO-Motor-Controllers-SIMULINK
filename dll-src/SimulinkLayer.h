#pragma once

#include <Windows.h>
#include <tchar.h>
#include <stdio.h>

#ifdef SOLOLIBRARY_EXPORTS
#define SOLOLIBRARY_API __declspec(dllexport)
#else
#define SOLOLIBRARY_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

	SOLOLIBRARY_API bool _serialSetup(UINT8 soloNum, char* _portName, UINT32 _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts);

	SOLOLIBRARY_API bool _SetDeviceAddress(UINT8 soloNum, unsigned char deviceAddress, int* error);

	SOLOLIBRARY_API bool _SetCommandMode(UINT8 soloNum, bool mode, int* error);

	SOLOLIBRARY_API bool _SetCurrentLimit(UINT8 soloNum, float currentLimit, int* error);

	SOLOLIBRARY_API bool _SetTorqueReferenceIq(UINT8 soloNum, float torqueReferenceIq, int* error);

	SOLOLIBRARY_API bool _SetSpeedReference(UINT8 soloNum, long speedReference, int* error);

	SOLOLIBRARY_API bool _SetPowerReference(UINT8 soloNum, float powerReference, int* error);

	SOLOLIBRARY_API bool _MotorParametersIdentification(UINT8 soloNum, bool identification, int* error);

	SOLOLIBRARY_API bool _EmergencyStop(UINT8 soloNum, int* error);

	SOLOLIBRARY_API bool _SetOutputPwmFrequencyKhz(UINT8 soloNum, long outputPwmFrequencyKhz, int* error);

	SOLOLIBRARY_API bool _SetSpeedControllerKp(UINT8 soloNum, float speedControllerKp, int* error);

	SOLOLIBRARY_API bool _SetSpeedControllerKi(UINT8 soloNum, float speedControllerKi, int* error);

	SOLOLIBRARY_API bool _SetMotorDirection(UINT8 soloNum, bool motorDirection, int* error);

	SOLOLIBRARY_API bool _SetMotorResistance(UINT8 soloNum, float motorResistance, int* error);

	SOLOLIBRARY_API bool _SetMotorInductance(UINT8 soloNum, float motorInductance, int* error);

	SOLOLIBRARY_API bool _SetMotorPolesCounts(UINT8 soloNum, long motorPolesCounts, int* error);

	SOLOLIBRARY_API bool _SetIncrementalEncoderLines(UINT8 soloNum, long incrementalEncoderLines, int* error);

	SOLOLIBRARY_API bool _SetSpeedLimit(UINT8 soloNum, long speedLimit, int* error);

	SOLOLIBRARY_API bool _ResetDeviceAddress(UINT8 soloNum, int* error);

	SOLOLIBRARY_API bool _SetFeedbackControlMode(UINT8 soloNum, UINT8 mode, int* error);

	SOLOLIBRARY_API bool _ResetFactory(UINT8 soloNum, int* error);

	SOLOLIBRARY_API bool _SetMotorType(UINT8 soloNum, UINT8 motorType, int* error);

	SOLOLIBRARY_API bool _SetControlMode(UINT8 soloNum, UINT8 controlMode, int* error);

	SOLOLIBRARY_API bool _SetCurrentControllerKp(UINT8 soloNum, float currentControllerKp, int* error);

	SOLOLIBRARY_API bool _SetCurrentControllerKi(UINT8 soloNum, float currentControllerKi, int* error);

	SOLOLIBRARY_API bool _SetMonitoringMode(UINT8 soloNum, bool mode, int* error);

	SOLOLIBRARY_API bool _SetMagnetizingCurrentIdReference(UINT8 soloNum, float magnetizingCurrentIdReference, int* error);

	SOLOLIBRARY_API bool _SetPositionReference(UINT8 soloNum, long positionReference, int* error);

	SOLOLIBRARY_API bool _SetPositionControllerKp(UINT8 soloNum, float positionControllerKp, int* error);

	SOLOLIBRARY_API bool _SetPositionControllerKi(UINT8 soloNum, float positionControllerKi, int* error);

	SOLOLIBRARY_API bool _ResetPositionToZero(UINT8 soloNum, int* error); //Home

	SOLOLIBRARY_API bool _OverwriteErrorRegister(UINT8 soloNum, int* error);

	SOLOLIBRARY_API bool _SetObserverGainBldcPmsm(UINT8 soloNum, float observerGain, int* error);

	SOLOLIBRARY_API bool _SetObserverGainBldcPmsmUltrafast(UINT8 soloNum, float observerGain, int* error);

	SOLOLIBRARY_API bool _SetObserverGainDc(UINT8 soloNum, float observerGain, int* error);

	SOLOLIBRARY_API bool _SetFilterGainBldcPmsm(UINT8 soloNum, float filterGain, int* error);

	SOLOLIBRARY_API bool _SetFilterGainBldcPmsmUltrafast(UINT8 soloNum, float filterGain, int* error);

	SOLOLIBRARY_API bool _SetUartBaudrate(UINT8 soloNum, bool baudrate, int* error);

	SOLOLIBRARY_API bool _SensorCalibration(UINT8 soloNum, UINT8 calibrationAction, int* error);

	SOLOLIBRARY_API bool _SetEncoderHallCcwOffset(UINT8 soloNum, float encoderHallOffset, int* error);

	SOLOLIBRARY_API bool _SetEncoderHallCwOffset(UINT8 soloNum, float encoderHallOffset, int* error);

	SOLOLIBRARY_API bool _SetSpeedAccelerationValue(UINT8 soloNum, float speedAccelerationValue, int* error);

	SOLOLIBRARY_API bool _SetSpeedDecelerationValue(UINT8 soloNum, float speedDecelerationValue, int* error);

	SOLOLIBRARY_API bool _SetCanbusBoudrate(UINT8 soloNum, UINT8 canbusBoudrate, int* error);

	SOLOLIBRARY_API float _GetCurrentLimit(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetDeviceAddress(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPhaseAVoltage(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPhaseBVoltage(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPhaseACurrent(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPhaseBCurrent(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetBusVoltage(UINT8 soloNum, int* error); //Battery Voltage

	SOLOLIBRARY_API float _GetDcMotorCurrentIm(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetDcMotorVoltageVm(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetSpeedControllerKp(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetSpeedControllerKi(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetOutputPwmFrequencyKhz(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetCurrentLimit(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetQuadratureCurrentIqFeedback(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetMagnetizingCurrentIdFeedback(UINT8 soloNum, int* error); //Magnetizing

	SOLOLIBRARY_API long  _GetMotorPolesCounts(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetIncrementalEncoderLines(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetCurrentControllerKp(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetCurrentControllerKi(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetBoardTemperature(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetMotorResistance(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetMotorInductance(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetSpeedFeedback(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetMotorType(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetFeedbackControlMode(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetCommandMode(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetControlMode(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetSpeedLimit(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPositionControllerKp(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPositionControllerKi(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetPositionCountsFeedback(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetErrorRegister(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetDeviceFirmwareVersion(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetDeviceHardwareVersion(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetTorqueReferenceIq(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetSpeedReference(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetMagnetizingCurrentIdReference(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetPositionReference(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetPowerReference(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetMotorDirection(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetObserverGainBldcPmsm(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetObserverGainBldcPmsmUltrafast(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetObserverGainDc(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetFilterGainBldcPmsm(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetFilterGainBldcPmsmUltrafast(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _Get3PhaseMotorAngle(UINT8 soloNum, int* error); // Read Estimated or Measured Rotor Angle

	SOLOLIBRARY_API float _GetEncoderHallCcwOffset(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetEncoderHallCwOffset(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetUartBaudrate(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetSpeedAccelerationValue(UINT8 soloNum, int* error);

	SOLOLIBRARY_API float _GetSpeedDecelerationValue(UINT8 soloNum, int* error);

	SOLOLIBRARY_API long  _GetEncoderIndexCounts(UINT8 soloNum, int* error);

	SOLOLIBRARY_API bool _serialIsWorking(UINT8 soloNum, int* error);

#ifdef __cplusplus
} // __cplusplus defined.
#endif
