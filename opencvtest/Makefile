CXX = g++
CXXFLAGS = -std=c++17 -O2 `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4`
TARGET = facedetect
SRC = facedetect.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)

