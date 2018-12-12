CPP=g++
CVFLAGS=`pkg-config --cflags --libs opencv`

DEPS=*.cpp
NAMEEXEC=application


$(NAMEEXEC): houghtransform.cpp
	echo "Building project ..."
	$(CPP) -o $@ $^ $(CVFLAGS)
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