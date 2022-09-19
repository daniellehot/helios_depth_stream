TARGET = main

OPENCV_LIB_PATH = $(shell pkg-config --libs-only-L opencv4)
OPENCV_INC_PATH = $(shell pkg-config --cflags-only-I opencv4)
OPENCV_LIBS = $(shell pkg-config --libs-only-l opencv4)

LDFLAGS = -L$(HOME)/ArenaSDK_Linux_x64/lib64 \
          -L$(HOME)/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64 \
          -L$(HOME)/ArenaSDK_Linux_x64/ffmpeg \
		  $(OPENCV_LIB_PATH)
          
GENICAMLIBS = -lGCBase_gcc54_v3_3_LUCID \
              -lGenApi_gcc54_v3_3_LUCID \
              -lLog_gcc54_v3_3_LUCID \
              -llog4cpp_gcc54_v3_3_LUCID \
              -lMathParser_gcc54_v3_3_LUCID \
              -lNodeMapData_gcc54_v3_3_LUCID \
              -lXmlParser_gcc54_v3_3_LUCID

CC=g++

INCLUDE= -I$(HOME)/ArenaSDK_Linux_x64/include/Arena \
         -I$(HOME)/ArenaSDK_Linux_x64/include/Save \
         -I$(HOME)/ArenaSDK_Linux_x64/include/GenTL \
         -I$(HOME)/ArenaSDK_Linux_x64/GenICam/library/CPP/include \
		 $(OPENCV_INC_PATH)

CFLAGS=-Wall -g -O2 -std=c++11 -Wno-unknown-pragmas

FFMPEGLIBS = -lavcodec \
             -lavformat \
             -lavutil \
             -lswresample

LIBS= -larena -lsave -lgentl $(GENICAMLIBS) $(FFMPEGLIBS) -lpthread -llucidlog $(OPENCV_LIBS)
RM = rm -f

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:%.cpp=%.o)
DEPS = $(OBJS:%.o=%.d)

.PHONY: all
all: ${TARGET}

${TARGET}: ${OBJS}
	${CC} ${LDFLAGS} $^ -o $@ $(LIBS)

%.o: %.cpp ${SRCS}
	${CC} ${INCLUDE}  ${LDFLAGS} -o $@ $< -c ${CFLAGS}

${DEPS}: %.cpp
	${CC} ${CLAGS} ${INCLUDE} -MM $< >$@

-include $(OBJS:.o=.d)

.PHONY: clean
clean:
	-${RM} ${TARGET} ${OBJS} ${DEPS}