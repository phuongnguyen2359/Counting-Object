INCLUDES = -I"c:\opencv\build\include" -I"c:\opencv\build\include\opencv" -I"c:\opencv\build\include\opencv2" 
-I"C:\Users\JackD\Downloads\Computing Engineering\final_counting_project"

CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 $(INCLUDES) -std=c++11 -lm

LIBS = -lopencv_core310.dll -lopencv_imgproc310.dll -lopencv_highgui310.dll \
-lopencv_ml310.dll -lopencv_video310.dll -lopencv_videoio310.dll -lopencv_features2d310.dll \
-lopencv_calib3d310.dll -lopencv_objdetect310.dll -lopencv_flann310.dll \
-lopencv_imgcodecs310.dll

LIBPATH =  -L"c:\opencv\build\x64\mingw_1\lib"

OBJS = final_counting_project.o detect.o

TARGET = final_counting_project.exe

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBPATH) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)