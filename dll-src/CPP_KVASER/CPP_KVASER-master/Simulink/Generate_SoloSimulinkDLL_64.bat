Del "dll_output/SoloDLL_Sim.dll"
Del "dll_output/SoloDLL_Sim.h"
g++ -o dll_output/SoloDLL_Sim.dll -I ..\inc -I ..\src -s -shared SimulinkLayer.cpp ..\src\*.cpp -L ..\Lib -lcanlib32 -Wl,--subsystem,windows
copy SimulinkLayer.h dll_output\SoloDLL_Sim.h