g++ -c -o SoloDLL_Sim.o SimulinkLayer.cpp  -D SOLOLIBRARY_EXPORTS
g++ -o SoloDLL_Sim.dll SoloDLL_Sim.o -s -shared -Wl,--subsystem,windows

g++ -shared -o libgeorgeringo.dll -Wl,—out-implib,libgeorgeringo.a -W1,—export-all-symbols-Wl,—enable-auto-image-base george.o ringo.o georgeringo.o