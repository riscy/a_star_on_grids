CC = g++ -O3 -Wall -std=c++11 -Wpadded
SRCFILES = graph.cpp heuristics.cpp algorithms.cpp benchmarks.cpp main.cpp
EXECUTABLE = main

.PHONY: run test
test: main
	./$(EXECUTABLE) --test

run: main
	./$(EXECUTABLE)

$(EXECUTABLE): $(SRCFILES)
	$(CC) -o $(EXECUTABLE) -lm $(SRCFILES)
