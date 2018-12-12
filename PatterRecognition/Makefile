CXX ?= g++

CXXFLAGS += -c -Wall $(shell pkg-config --cflags opencv)
LDFLAGS += $(shell pkg-config --libs --static opencv)

NAMEFILE=laplacian
all: exe

exe: $(NAMEFILE).o; $(CXX) $< -o $@ $(LDFLAGS)

%.o: %.cpp; $(CXX) $< -o $@ $(CXXFLAGS)

#opencv_example_test: test.o; $(CXX) $< -o $@ $(LDFLAGS)


clean: ; rm -f .o exe

exec:
	./exe
.PHONY=exec
