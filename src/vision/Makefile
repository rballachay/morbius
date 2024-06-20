CXX = g++
CXXFLAGS = -std=c++11 `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`

all: realsense-example

realsense-example: realsense.o
    $(CXX) -o realsense-example.cpp realsense.o $(LDFLAGS)

realsense.o: realsense-example.cpp
    $(CXX) -c realsense-example.cpp $(CXXFLAGS)

clean:
    rm -f realsense-example realsense.o
