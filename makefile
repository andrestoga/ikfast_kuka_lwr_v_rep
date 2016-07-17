CFLAGS = -I../include -I. -lstdc++ -Wall -fPIC -static

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
else
	CFLAGS += -D__APPLE__
	EXT = dylib
endif

all: 
	@rm -f lib/*.$(EXT)
	@rm -f *.o 
	g++ $(CFLAGS) -c v_repExtLWRIKFast.cpp -o v_repExtLWRIKFast.o
	g++ $(CFLAGS) -c lwr_ikfast.cpp -o lwr_ikfast.o
	g++ $(CFLAGS) -c IKFastKinematicsPlugin.cpp -o IKFastKinematicsPlugin.o
	g++ $(CFLAGS) -c ../common/luaFunctionData.cpp -o luaFunctionData.o
	g++ $(CFLAGS) -c ../common/luaFunctionDataItem.cpp -o luaFunctionDataItem.o
	g++ $(CFLAGS) -c ../common/v_repLib.cpp -o v_repLib.o
	@mkdir -p lib
	g++ lwr_ikfast.o IKFastKinematicsPlugin.o luaFunctionData.o luaFunctionDataItem.o v_repExtLWRIKFast.o v_repLib.o -o lib/libv_repExtLWRIKFast.$(EXT) -lpthread -ldl -shared 

