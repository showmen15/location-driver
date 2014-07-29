CXX = g++
CXXFLAGS = -static -pedantic -W -Wall -Wextra -Wshadow -Wformat \
	-Winit-self -Wunused -Wfloat-equal -Wcast-qual -Wwrite-strings \
	-Winline -Wstack-protector -Wunsafe-loop-optimizations \
	-Wlogical-op -Wmissing-include-dirs -Wconversion \
	-Wmissing-declarations -Wno-long-long

LDFLAGS = -lm
PROTOC = protoc
PROTOC_FLAGS = 

CPP_FILES := $(wildcard *.cpp)
OBJS := $(patsubst %.cpp, %.o, $(CPP_FILES))
PROTO_FILES := $(wildcard *.proto)
PROTO_CC_FILES := $(patsubst %.proto, %.pb.cc, $(PROTO_FILES))
PROTO_H_FILES := $(patsubst %.proto, %.pb.h, $(PROTO_FILES))
PROTO_OBJ_FILES := $(patsubst %.proto, %.pb.o, $(PROTO_FILES))

all: protobuf $(OBJS)
	
protobuf: $(PROTO_OBJ_FILES) $(PROTO_H_FILES) 

$(PROTO_H_FILES): $(PROTO_FILES)
	$(PROTOC) --cpp_out=. $(PROTOC_FLAGS) $<

$(PROTO_CC_FILES): $(PROTO_FILES)
	$(PROTOC) --cpp_out=. $(PROTOC_FLAGS) $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.pb.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(PROTO_H_FILES) $(PROTO_CC_FILES) $(PROTO_OBJ_FILES)

.PHONY: clean protobuf