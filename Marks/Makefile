INCLUDE=./include/
OPENCV_LIBS=`pkg-config --libs opencv`
CFLAGS = `pkg-config opencv --cflags`
CLIBS =  `pkg-config opencv --libs`
INCLUDEDIR=-I$(INCLUDE) -I/usr/local/include/opencv -I/usr/local/include -I. $(CFLAGS)
LIBDIRS=-L/usr/local/lib/
LIBS =  -lm $(OPENCV_LIBS) -lrt -lm -ldl -pthread
CFLAGS2   = -g -Wno-deprecated -ansi -Wall 
LDFLAGS = -lm -lblas -llapack -lrt

#all:testCoor testMarcas
all:testMarks

testMarks: objs/marks.o 
	g++ -o $@ $^ $(CFLAGS) $(LDFLAGS) $(CLIBS) $(LIBDIRS) $(LIBS)

objs/%.o: %.cpp 
	g++ -fPIE -c -o $@ $< $(CLIBS) $(INCLUDEDIR) #$(LIBDIRS) $(LIBS)

clean:
	rm ./objs/* testMarks

#the -o $@ says to put the output of the compilation in the file named on the left side of the :
 #the special macros $@ and $^, which are the left and right sides of the :
 #$< The name of the first prerequisite.
