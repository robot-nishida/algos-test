# Makefile

main: main.o odetools.o
	g++ -o exe main.o odetools.o `ode-config --cflags --libs` -ldrawstuff -framework GLUT -framework OpenGL

main.o: main.cpp
	g++ -c main.cpp -I ~/Software/local/include/

odetools.o: odetools.cpp
	g++ -c odetools.cpp -I ~/Software/local/include/

main.o: odetools.h

odetools.o: odetools.h

.PHONY: clean
clean:
	rm -f main main.o odetools.o
