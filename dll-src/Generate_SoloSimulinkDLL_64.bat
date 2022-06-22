g++ -c -o SoloDLL_Sim.o SimulinkLayer.cpp  -D SOLOLIBRARY_EXPORTS
g++ -o SoloDLL_Sim.dll SoloDLL_Sim.o -s -shared -Wl,--subsystem,windows