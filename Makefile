# should be either OSC_HOST_BIG_ENDIAN or OSC_HOST_LITTLE_ENDIAN
# Apple: OSC_HOST_BIG_ENDIAN
# Win32: OSC_HOST_LITTLE_ENDIAN
# i386 LinuX: OSC_HOST_LITTLE_ENDIAN

ENDIANESS=OSC_HOST_LITTLE_ENDIAN
PLATFORM=$(shell uname)

SDL_CFLAGS  := $(shell sdl-config --cflags)
SDL_LDFLAGS := $(shell sdl-config --libs) -lpthread

TUIO_SENDER = TUIOSender
TUIO_STATIC  = libTUIO.a
TUIO_SHARED  = libTUIO.so
OPENCV_STREAM = stream_or_pictures
KALIBRATION_STREAM = kalibration

INCLUDES = -I./libbaumer/src/baumer/inc -I./tuio -I./oscpack -I/opt/boost/boost_1_55_0/include -Ipng++-0.2.3/ -I/opt/boost/boost_1_55_0/include -I./opencv-touch/src
LDPATH = -L./opencv-touch/src -L/opt/boost/boost_1_55_0/lib -L/usr/local/lib/baumer
LDFLAGS = -lbgapi2_ext \
          -lbgapi2_genicam \
          -lbgapi2_img \
          -lcamera_tools \
          -levisionlib \
          -limage_tools \
          -lMathParser \
          -lrt -lboost_thread -ljpeg -lpng -lGL -lglut -lpthread -lbgapi \
          -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_video -lboost_system \
          -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_objdetect
          



CFLAGS  = -fPIC -Wall -O3 $(SDL_CFLAGS) -DLINUX -D_GNULINUX
#CFLAGS  = -g -Wall -O3 $(SDL_CFLAGS)
CXXFLAGS = $(CFLAGS) $(INCLUDES) -D$(ENDIANESS) -std=c++11
SHARED_OPTIONS = -shared -Wl,-soname,$(TUIO_SHARED)

TUIO_SENDER_SOURCES = TUIOSender.cpp
TUIO_SENDER_OBJECTS = TUIOSender.o
#TUIO_SENDER_OBJECTS = /home/veva6054/Desktop/TuioFakeInputs/cpp/TUIOServer/TUIOSender.o

TUIO_SOURCES = ./tuio/TuioClient.cpp ./tuio/TuioServer.cpp ./tuio/TuioTime.cpp
OSC_SOURCES = ./oscpack/osc/OscTypes.cpp ./oscpack/osc/OscOutboundPacketStream.cpp ./oscpack/osc/OscReceivedElements.cpp ./oscpack/osc/OscPrintReceivedElements.cpp ./oscpack/ip/posix/NetworkingUtils.cpp ./oscpack/ip/posix/UdpSocket.cpp

OCV_SOURCES = ./stream_or_pictures.cpp
OCV_OBJECTS = ./stream_or_pictures.o

KALIBRATION_SOURCES = ./kalibration.cpp
KALIBRATION_OBJECTS = ./kalibration.o

COMMON_SOURCES = $(TUIO_SOURCES) $(OSC_SOURCES)
COMMON_OBJECTS = $(COMMON_SOURCES:.cpp=.o)


all: tuiosender opencv static libbaumer shared

static:	$(COMMON_OBJECTS)
	ar rcs $(TUIO_STATIC) $(COMMON_OBJECTS)

shared:	$(COMMON_OBJECTS)
	$(CXX) -lpthread $+ $(SHARED_OPTIONS) -o $(TUIO_SHARED)

tuiosender:	$(COMMON_OBJECTS) $(TUIO_SENDER_OBJECTS) $(KALIBRATION_OBJECTS)
	$(CXX) -o $(TUIO_SENDER) $+ $(SDL_LDFLAGS) $(FRAMEWORKS) $(LDPATH) $(LDFLAGS)

opencv: $(COMMON_OBJECTS) $(KALIBRATION_OBJECTS) $(TUIO_SENDER_OBJECTS)
	$(CXX) -o $(KALIBRATION_STREAM) $+ $(SDL_LDFLAGS) $(FRAMEWORKS) $(LDPATH) $(LDFLAGS)

libbaumer:
	cd ./opencv-touch/src/ && make

clean:
	rm -rf $(TUIO_SENDER) $(KALIBRATION_STREAM) $(TUIO_STATIC) $(TUIO_SHARED) $(COMMON_OBJECTS) $(OCV) $(DISTORTION_OBJECTS) $(TUIO_SENDER_OBJECTS)
	cd ./opencv-touch/src/ && make clean
