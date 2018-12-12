CPP=g++
CVFLAGS=`pkg-config --cflags --libs opencv`

DEPS=*.cpp
NAMEEXEC=application


$(NAMEEXEC): RealTimeNoiseDeletion.cpp
	echo "Building project ..."
	$(CPP) -g -o $@ $^ $(CVFLAGS)
	echo "Successfull building"	

clean:
	rm application
.PHONY: clean

all:
	make clean
	make $(NAMEEXEC)
	./$(NAMEEXEC)

.PHONY: all

exec:
	./$(NAMEEXEC)