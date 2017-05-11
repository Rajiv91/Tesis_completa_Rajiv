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
all:testMarcas

testCoor: objs/coordenadas.o
	g++ -o $@ $^ $(CFLAGS) $(CLIBS) $(LIBDIRS) $(LIBS)


testMarcas: objs/marcas.o objs/tools.o $(INCLUDE)Tools.h
	g++ -o $@ $^ $(CFLAGS) $(LDFLAGS) $(CLIBS) $(LIBDIRS) $(LIBS)

#we define a rule that applies to all files ending in the .o suffix. The rule says that the .o file depends upon the .c version of the file
#se deben incluir los header files sino hay que estar borrando los .o cada vez que se haga un cambio para compilar
objs/%.o: %.cpp  $(INCLUDE)Tools.h $(INCLUDE)Tools2.h $(INCLUDE)functions.h $(INCLUDE)levenberg.h
	g++ -fPIE -c -o $@ $< $(CLIBS) $(INCLUDEDIR) #$(LIBDIRS) $(LIBS)
	#g++ -I$(INCLUDE) -c -o objs/main.o main.cpp #Igual funciona
#-I$(INCLUDE)

objs/tools.o: $(INCLUDE)tools.cpp  $(INCLUDE)tools.h 
	g++ -O2 -c -o  $@ $< $(CFLAGS2)  
clean:
	rm ./objs/* testCoor testMarcas

#the -o $@ says to put the output of the compilation in the file named on the left side of the :
 #the special macros $@ and $^, which are the left and right sides of the :
 #$< The name of the first prerequisite.
