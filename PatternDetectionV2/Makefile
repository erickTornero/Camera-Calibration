CPP=g++
CVFLAGS=`pkg-config --cflags --libs opencv`

DEPS=*.cpp
NAMEEXEC=application


$(NAMEEXEC): houghtransform.cpp
	echo "Building project ..."
	$(CPP) -o $@ $^ $(CVFLAGS)
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