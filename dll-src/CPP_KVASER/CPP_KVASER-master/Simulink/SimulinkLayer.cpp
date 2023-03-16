#include "SimulinkLayer.h"
#include "SOLOMotorControllers.h"
#include "SOLOMotorControllersSerial.h"
#include "SOLOMotorControllersKvaser.h"

SOLOMotorControllers *solo[256];
bool soloInit[256];

bool _Setup(UINT8 soloNum, UINT8 connectionType, char* _portName, UINT32 _baudrate, long _millisecondsTimeout, int _packetFailureTrialAttempts)
{
	if(connectionType == 0)
	{
		int baud;
		solo[soloNum] = new SOLOMotorControllersKvaser();
		soloInit[soloNum] = true;
		switch(_baudrate)
		{
			case 1000:
				baud = -1;
				break;
			case 500:
				baud = -2;
				break;
			case 250:
				baud = -3;
				break;
			case 125:
				baud = -4;
				break;
			case 100:
				baud = -5;
				break;
			default:
				baud = -1;
				break;	
		}
		return solo[soloNum]->Connect(soloNum, _portName, baud, _millisecondsTimeout, _packetFailureTrialAttempts);
	}
	else if(connectionType == 1)
	{
		solo[soloNum] = new SOLOMotorControllersSerial();
		soloInit[soloNum] = true;
		return solo[soloNum]->Connect(soloNum, _portName, _baudrate, _millisecondsTimeout, _packetFailureTrialAttempts);
	}
	else
		return false;
}

bool _SetDeviceAddress(UINT8 soloNum, unsigned char deviceAddress, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetDeviceAddress(deviceAddress, *error);
}

bool _SetCommandMode(UINT8 soloNum, bool mode, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::CommandMode m;
	if (mode)
		m = SOLOMotorControllers::digital;
	else
		m = SOLOMotorControllers::analogue;
	return solo[soloNum]->SetCommandMode(m, *error);
}

bool _SetCurrentLimit(UINT8 soloNum, float currentLimit, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetCurrentLimit(currentLimit, *error);
}

bool _SetTorqueReferenceIq(UINT8 soloNum, float torqueReferenceIq, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetTorqueReferenceIq(torqueReferenceIq, *error);
}

bool _SetSpeedReference(UINT8 soloNum, long speedReference, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedReference(speedReference, *error);
}

bool _SetPowerReference(UINT8 soloNum, float powerReference, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetPowerReference(powerReference, *error);
}

bool _MotorParametersIdentification(UINT8 soloNum, bool identification, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::Action act;
	if (identification)
		act = SOLOMotorControllers::start;
	else
		act = SOLOMotorControllers::stop;

	return solo[soloNum]->MotorParametersIdentification(act, *error);
}

bool _EmergencyStop(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->EmergencyStop(*error);
}

bool _SetOutputPwmFrequencyKhz(UINT8 soloNum, long outputPwmFrequencyKhz, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetOutputPwmFrequencyKhz(outputPwmFrequencyKhz, *error);
}

bool _SetSpeedControllerKp(UINT8 soloNum, float speedControllerKp, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedControllerKp(speedControllerKp, *error);
}

bool _SetSpeedControllerKi(UINT8 soloNum, float speedControllerKi, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedControllerKi(speedControllerKi, *error);
}

bool _SetMotorDirection(UINT8 soloNum, bool motorDirection, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::Direction dir;
	if (motorDirection)
		dir = SOLOMotorControllers::counterclockwise;
	else
		dir = SOLOMotorControllers::clockwise;

	return solo[soloNum]->SetMotorDirection(dir, *error);
}

bool _SetMotorResistance(UINT8 soloNum, float motorResistance, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetMotorResistance(motorResistance, *error);
}

bool _SetMotorInductance(UINT8 soloNum, float motorInductance, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetMotorInductance(motorInductance, *error);
}

bool _SetMotorPolesCounts(UINT8 soloNum, long motorPolesCounts, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetMotorPolesCounts(motorPolesCounts, *error);
}

bool _SetIncrementalEncoderLines(UINT8 soloNum, long incrementalEncoderLines, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetIncrementalEncoderLines(incrementalEncoderLines, *error);
}

bool _SetSpeedLimit(UINT8 soloNum, long speedLimit, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedLimit(speedLimit, *error);
}

bool _ResetDeviceAddress(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->ResetDeviceAddress(*error);
}

bool _SetFeedbackControlMode(UINT8 soloNum, UINT8 mode, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::FeedbackControlMode fb;
	switch (mode)
	{
	case 0:
		fb = SOLOMotorControllers::sensorLess;
		break;
	case 1:
		fb = SOLOMotorControllers::encoders;
		break;
	case 2:
		fb = SOLOMotorControllers::hallSensors;
		break;
	default:
		return false;
		break;
	}

	return solo[soloNum]->SetFeedbackControlMode(fb , *error);
}

bool _ResetFactory(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->ResetFactory(*error);
}

bool _SetMotorType(UINT8 soloNum, UINT8 motorType, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::MotorType mType;
	switch (motorType)
	{
	case 0:
		mType = SOLOMotorControllers::dc;
		break;
	case 1:
		mType = SOLOMotorControllers::bldcPmsm;
		break;
	case 2:
		mType = SOLOMotorControllers::acim;
		break;
	case 3:
		mType = SOLOMotorControllers::bldcPmsmUltrafast;
		break;
	default:
		return false;
		break;
	}

	return solo[soloNum]->SetMotorType(mType, *error);
}

bool _SetControlMode(UINT8 soloNum, UINT8 controlMode, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::ControlMode cntrl;
	switch (controlMode)
	{
	case 0:
		cntrl = SOLOMotorControllers::speedMode;
		break;
	case 1:
		cntrl = SOLOMotorControllers::torqueMode;
		break;
	case 2:
		cntrl = SOLOMotorControllers::positionMode;
		break;
	default:
		return false;
		break;
	}

	return solo[soloNum]->SetControlMode(cntrl, *error);
}

bool _SetCurrentControllerKp(UINT8 soloNum, float currentControllerKp, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetCurrentControllerKp(currentControllerKp, *error);
}

bool _SetCurrentControllerKi(UINT8 soloNum, float currentControllerKi, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetCurrentControllerKi(currentControllerKi, *error);
}

bool _SetMonitoringMode(UINT8 soloNum, bool mode, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetMonitoringMode(mode, *error);
}

bool _SetMagnetizingCurrentIdReference(UINT8 soloNum, float magnetizingCurrentIdReference, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetMagnetizingCurrentIdReference(magnetizingCurrentIdReference, *error);
}

bool _SetPositionReference(UINT8 soloNum, long positionReference, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetPositionReference(positionReference, *error);
}

bool _SetPositionControllerKp(UINT8 soloNum, float positionControllerKp, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetPositionControllerKp(positionControllerKp, *error);
}

bool _SetPositionControllerKi(UINT8 soloNum, float positionControllerKi, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetPositionControllerKi(positionControllerKi, *error);
}

bool _ResetPositionToZero(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->ResetPositionToZero(*error);
}

bool _OverwriteErrorRegister(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->OverwriteErrorRegister(*error);
}

bool _SetObserverGainBldcPmsm(UINT8 soloNum, float observerGain, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetObserverGainBldcPmsm(observerGain, *error);
}

bool _SetObserverGainBldcPmsmUltrafast(UINT8 soloNum, float observerGain, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetObserverGainBldcPmsmUltrafast(observerGain, *error);
}

bool _SetObserverGainDc(UINT8 soloNum, float observerGain, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetObserverGainDc(observerGain, *error);
}

bool _SetFilterGainBldcPmsm(UINT8 soloNum, float filterGain, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetFilterGainBldcPmsm(filterGain, *error);
}

bool _SetFilterGainBldcPmsmUltrafast(UINT8 soloNum, float filterGain, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetFilterGainBldcPmsmUltrafast(filterGain, *error);
}

bool _SetUartBaudrate(UINT8 soloNum, bool baudrate, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::UartBaudrate baud;
	if (baudrate)
		baud = SOLOMotorControllers::rate115200;
	else
		baud = SOLOMotorControllers::rate937500;

	return solo[soloNum]->SetUartBaudrate(baud, *error);
}

bool _SensorCalibration(UINT8 soloNum, UINT8 calibrationAction, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::PositionSensorCalibrationAction calibAct;
	switch (calibrationAction)
	{
	case 0:
		calibAct = SOLOMotorControllers::stopCalibration;
		break;
	case 1:
		calibAct = SOLOMotorControllers::incrementalEncoderStartCalibration;
		break;
	case 2:
		calibAct = SOLOMotorControllers::hallSensorStartCalibration;
		break;
	default:
		return false;
		break;
	}

	return solo[soloNum]->SensorCalibration(calibAct, *error);
}

bool _SetEncoderHallCcwOffset(UINT8 soloNum, float encoderHallOffset, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetEncoderHallCcwOffset(encoderHallOffset, *error);
}

bool _SetEncoderHallCwOffset(UINT8 soloNum, float encoderHallOffset, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetEncoderHallCwOffset(encoderHallOffset, *error);
}

bool _SetSpeedAccelerationValue(UINT8 soloNum, float speedAccelerationValue, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedAccelerationValue(speedAccelerationValue, *error);
}

bool _SetSpeedDecelerationValue(UINT8 soloNum, float speedDecelerationValue, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->SetSpeedDecelerationValue(speedDecelerationValue, *error);
}

bool _SetCanbusBaudrate(UINT8 soloNum, UINT8 canbusBaudrate, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	SOLOMotorControllers::CanbusBaudrate canBaud;
	switch (canbusBaudrate)
	{
	case 0:
		canBaud = SOLOMotorControllers::rate1000;
		break;
	case 1:
		canBaud = SOLOMotorControllers::rate500;
		break;
	case 2:
		canBaud = SOLOMotorControllers::rate250;
		break;
	case 3:
		canBaud = SOLOMotorControllers::rate125;
		break;
	case 4:
		canBaud = SOLOMotorControllers::rate100;
		break;
	default:
		return false;
		break;
	}

	return solo[soloNum]->SetCanbusBaudrate(canBaud, *error);
}

long  _GetDeviceAddress(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetDeviceAddress(*error);
}

float _GetPhaseAVoltage(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPhaseAVoltage(*error);
}

float _GetPhaseBVoltage(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPhaseBVoltage(*error);
}

float _GetPhaseACurrent(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPhaseACurrent(*error);
}

float _GetPhaseBCurrent(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPhaseBCurrent(*error);
}

float _GetBusVoltage(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetBusVoltage(*error);
}

float _GetDcMotorCurrentIm(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetDcMotorCurrentIm(*error);
}

float _GetDcMotorVoltageVm(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetDcMotorVoltageVm(*error);
}

float _GetSpeedControllerKp(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedControllerKp(*error);
}

float _GetSpeedControllerKi(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedControllerKi(*error);
}

long  _GetOutputPwmFrequencyKhz(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetOutputPwmFrequencyKhz(*error);
}

float _GetCurrentLimit(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetCurrentLimit(*error);
}

float _GetQuadratureCurrentIqFeedback(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetQuadratureCurrentIqFeedback(*error);
}

float _GetMagnetizingCurrentIdFeedback(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetQuadratureCurrentIqFeedback(*error);
}

long  _GetMotorPolesCounts(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMotorPolesCounts(*error);
}

long  _GetIncrementalEncoderLines(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetIncrementalEncoderLines(*error);
}

float _GetCurrentControllerKp(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetCurrentControllerKp(*error);
}

float _GetCurrentControllerKi(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetCurrentControllerKi(*error);
}

float _GetBoardTemperature(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetBoardTemperature(*error);
}

float _GetMotorResistance(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMotorResistance(*error);
}

float _GetMotorInductance(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMotorInductance(*error);
}

long  _GetSpeedFeedback(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedFeedback(*error);
}

long  _GetMotorType(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMotorType(*error);
}

long  _GetFeedbackControlMode(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetFeedbackControlMode(*error);
}

long  _GetCommandMode(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetCommandMode(*error);
}

long  _GetControlMode(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetControlMode(*error);
}

long  _GetSpeedLimit(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedLimit(*error);
}

float _GetPositionControllerKp(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPositionControllerKp(*error);
}

float _GetPositionControllerKi(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPositionControllerKi(*error);
}

long  _GetPositionCountsFeedback(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPositionCountsFeedback(*error);
}

long  _GetErrorRegister(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetErrorRegister(*error);
}

long  _GetDeviceFirmwareVersion(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetDeviceFirmwareVersion(*error);
}

long  _GetDeviceHardwareVersion(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetDeviceHardwareVersion(*error);
}

float _GetTorqueReferenceIq(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetTorqueReferenceIq(*error);
}

long  _GetSpeedReference(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedReference(*error);
}

float _GetMagnetizingCurrentIdReference(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMagnetizingCurrentIdReference(*error);
}

long  _GetPositionReference(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPositionReference(*error);
}

float _GetPowerReference(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetPowerReference(*error);
}

long  _GetMotorDirection(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetMotorDirection(*error);
}

float _GetObserverGainBldcPmsm(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetObserverGainBldcPmsm(*error);
}

float _GetObserverGainBldcPmsmUltrafast(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetObserverGainBldcPmsmUltrafast(*error);
}

float _GetObserverGainDc(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetObserverGainDc(*error);
}

float _GetFilterGainBldcPmsm(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetFilterGainBldcPmsm(*error);
}

float _GetFilterGainBldcPmsmUltrafast(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetFilterGainBldcPmsmUltrafast(*error);
}

float _Get3PhaseMotorAngle(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->Get3PhaseMotorAngle(*error);
}

float _GetEncoderHallCcwOffset(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetEncoderHallCcwOffset(*error);
}

float _GetEncoderHallCwOffset(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetEncoderHallCwOffset(*error);
}

long  _GetUartBaudrate(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetUartBaudrate(*error);
}

float _GetSpeedAccelerationValue(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedAccelerationValue(*error);
}

float _GetSpeedDecelerationValue(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetSpeedDecelerationValue(*error);
}

long  _GetEncoderIndexCounts(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return 0;
	}
	return solo[soloNum]->GetEncoderIndexCounts(*error);
}

bool _CommunicationIsWorking(UINT8 soloNum, int* error)
{
	if(!soloInit[soloNum])
	{
		*error = SOLOMotorControllers::Error::objectNotInitialize;
		return false;
	}
	return solo[soloNum]->CommunicationIsWorking(*error);
}