CC = g++

CFLAGS = `pkg-config --cflags opencv`

LIBS = `pkg-config --libs opencv`

testElevator : testElevator.cpp
	$(CC) -o testElevator testElevator.cpp $(CFLAGS) $(LIBS) 

clean:
	$(RM) -f testElevator