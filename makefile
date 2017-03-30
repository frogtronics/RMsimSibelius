COMMON=/O2 /MT /EHsc /I../include ../bin/mujoco140.lib /Fe../RMsimSibelius/

all:
	cl $(COMMON) sim_render.cpp    ../bin/glfw3.lib
	cl $(COMMON) sim_replay.cpp    ../bin/glfw3.lib
	cl $(COMMON) rsim.cpp    ../bin/glfw3.lib
	del *.obj
