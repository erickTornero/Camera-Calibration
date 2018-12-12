CPP=g++
CVFLAGS=`pkg-config --cflags --libs opencv`

DEPS=*.cpp
NAMEEXEC=application


$(NAMEEXEC): RealTimeNoiseDeletion.cpp
	echo "Building project ..."
	$(CPP) -g -o $@ $^ $(CVFLAGS)
	echo "Successfull building"	

clean:
	rm -f application
.PHONY: clean

all:
	make $(NAMEEXEC)
	./$(NAMEEXEC)

.PHONY: all

exec:
	./$(NAMEEXEC)