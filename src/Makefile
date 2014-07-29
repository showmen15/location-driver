CXX = g++
CXXFLAGS = -g -static -pedantic -W -Wall -Wextra -Wshadow -Wformat \
	-Winit-self -Wunused -Wfloat-equal -Wcast-qual -Wwrite-strings \
	-Winline -Wstack-protector -Wunsafe-loop-optimizations \
	-Wlogical-op -Wmissing-include-dirs -Wconversion \
	-Wmissing-declarations -Wno-long-long

LDFLAGS = -lrt -lpthread -lboost_thread -lprotobuf -llog4cxx -lboost_program_options

EXECUTABLE = location_driver

BINDIR = ../bin/

AMBER_COMMON = ../../common
AMBER_COMMON_OBJS = $(wildcard $(AMBER_COMMON)/*.o)

INCLUDES = -I$(AMBER_COMMON)

PROTOC = protoc
PROTOC_FLAGS = -I$(AMBER_COMMON) -I. 

CPP_FILES := $(wildcard *.cpp)
OBJS := $(patsubst %.cpp, %.o, $(CPP_FILES))
PROTO_FILES := $(wildcard *.proto)
PROTO_CC_FILES := $(patsubst %.proto, %.pb.cc, $(PROTO_FILES))
PROTO_H_FILES := $(patsubst %.proto, %.pb.h, $(PROTO_FILES))
PROTO_OBJ_FILES := $(patsubst %.proto, %.pb.o, $(PROTO_FILES))

all: $(EXECUTABLE)

$(EXECUTABLE): $(PROTO_OBJ_FILES) $(PROTO_H_FILES) $(OBJS)
	test -d $(BINDIR) || mkdir $(BINDIR)
	$(CXX)  $(OBJS) $(AMBER_COMMON_OBJS) $(PROTO_OBJ_FILES) $(LDFLAGS) -o $(BINDIR)$@ 

$(PROTO_H_FILES): $(PROTO_FILES)
	$(PROTOC) --cpp_out=. $(PROTOC_FLAGS) $<

$(PROTO_CC_FILES): $(PROTO_FILES)
	$(PROTOC) --cpp_out=. $(PROTOC_FLAGS) $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

%.pb.o: %.pb.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJS) $(PROTO_H_FILES) $(PROTO_CC_FILES) $(PROTO_OBJ_FILES) $(BINDIR)

.PHONY: clean
