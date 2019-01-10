CPP=g++
CVFLAGS=`pkg-config --cflags --libs opencv`

DEPS=*.cpp
NAMEEXEC=application


$(NAMEEXEC): main.cpp
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

acircles1:
	./application -w=4 -h=11 -s=38 -o=acircles1.yml -V 'circulos2.avi' -oe -op -pt=acircles
chess1:
	./application -w=4 -h=11 -s=26 -o=chess1.yml -V 'circulos2.avi' -oe -op -pt=chessboard

acircles2:
	./application -w=4 -h=11 -s=38 -o=acircles2.yml -V 'circulos2.avi' -oe -op -pt=acircles
chess2:
	./application -w=4 -h=11 -s=26 -o=chess2.yml -V 'circulos2.avi' -oe -op -pt=chessboard

.PHONY: all

exec:
	./$(NAMEEXEC)
