# List of demos to compile and link
DEVLIST=109

CXXFLAGS=-I. -I../../inc
CLFLAGS=$(DEVLIST:%=-leci%DriverLinux)
CXX=gcc

default: LinuxEciDemo

LinuxEciDemo: $(DEVLIST:%=EciDemo%.o) EciDemoCommon.o LinuxEciDemo.o
	$(CXX) -o ../../bin/release/LinuxEciDemo LinuxEciDemo.o EciDemoCommon.o $(DEVLIST:%=EciDemo%.o) $(CLFLAGS)

LinuxEciDemo.o: LinuxEciDemo/LinuxEciDemo.c
	$(CXX) -c $(CXXFLAGS) LinuxEciDemo/LinuxEciDemo.c
  
EciDemoCommon.o: EciDemoCommon.c EciDemoCommon.h
	$(CXX) -c $(CXXFLAGS) EciDemoCommon.c

$(DEVLIST:%=EciDemo%.o): $(DEVLIST:%=EciDemo%.c) $(DEVLIST:%=EciDemo%.h)
	$(CXX) -c $(CXXFLAGS) $(DEVLIST:%=EciDemo%.c)
  
clean:
	rm -f *.o ../../bin/release/LinuxEciDemo
