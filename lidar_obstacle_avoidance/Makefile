CX = g++    # c++ 코드이면 g++
CXFLAGS = -Wall  # debug mode, 모든경고 출력
CXFLAGS += -I/home/ncslab/rplidar_sdk/sdk/include -I/home/ncslab/rplidar_sdk/sdk/src -I/usr/local/include/dynamixel_sdk_cpp
LD_LIBS = -lstdc++ -lpthread -lrt -ldxl_x64_cpp -lrt
LD_LIBS += -L/home/ncslab/rplidar_sdk/output/Linux/Release/ -lsl_lidar_sdk
LIBS += `pkg-config opencv4 --cflags --libs` 

CXSRC = main.cpp
TARGET = lidar
OBJS = main.o lidar.o dxl.o
$(TARGET) :  $(OBJS)
	$(CX) $(CXFLAGS) -o $(TARGET) $(OBJS) $(LD_LIBS) $(LIBS)
main.o : main.cpp dxl.hpp
	$(CX) $(CXFLAGS) -c main.cpp $(LD_LIBS) $(LIBS) $(DXLFLAGS) 
lidar.o : lidar.cpp lidar.hpp
	$(CX) $(CXFLAGS) -c lidar.cpp $(LD_LIBS) $(LIBS)
dxl.o : dxl.hpp dxl.cpp
	$(CX) $(CXFLAGS) -c dxl.cpp 
.PHONY: all clean

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)

#echo:
#	@echo $(CXFLAGS)
#	@echo $(LD_LIBS)
#	@echo $(TARGET)
