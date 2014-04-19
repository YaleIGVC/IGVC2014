CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS := '-L/IGVC2014/src/mako_grabber/VimbaCPP/DynamicLib/x86_64bit'
CC_FLAGS := '-I.'

main: $(OBJ_FILES)
	   g++ $(LD_FLAGS) -o $@ $^

obj/%.o: src/%.cpp
	   g++ $(CC_FLAGS) -c -o $@ $<
