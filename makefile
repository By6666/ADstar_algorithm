exe: main.o ADstar_algorithm.o grid_input.o
	g++ -o exe.out main.o ADstar_algorithm.o grid_input.o

main.o: main.cpp
	g++ -c main.cpp -std=c++11

ADstar_algorithm.o: ADstar_algorithm.cpp
	g++ -c ADstar_algorithm.cpp -std=c++11

grid_input.o: include/grid_input.cpp
	g++ -c include/grid_input.cpp

clean:
	rm -f *.o
