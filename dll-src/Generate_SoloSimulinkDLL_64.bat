Del "../s-function/SoloDLL_Sim.dll"
Del "../s-function/SoloDLL_Sim.h"
g++ -o ../s-function/SoloDLL_Sim.dll -s -shared SimulinkLayer.cpp SoloMotorControllers.cpp -Wl,--subsystem,windows
cp SimulinkLayer.h ../s-function/SoloDLL_Sim.h