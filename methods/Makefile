# Compiler settings
CXX = g++
CXXFLAGS = -O3 -std=c++17 -I../include
OMPFLAGS = -O3 -std=c++17 -fopenmp -I../include

# Sources and outputs
METHOD1_SRC = method-1/main.cpp
METHOD1_OUT = method-1/exe.out

METHOD2_SRC = method-2/main.cpp
METHOD2_OUT = method-2/exe.out

METHOD3_SRC = method-3/main.cpp
METHOD3_OUT = method-3/exe.out

METHOD3_MT_SRC = method-3-multithreaded-v1/main.cpp
METHOD3_MT_OUT = method-3-multithreaded-v1/exe.out

METHOD4_SRC = method-4/main.cpp
METHOD4_OUT = method-4/exe.out

METHOD5_SRC = method-5/main.cpp
METHOD5_OUT = method-5/exe.out

METHOD5_MT_SRC = method-5-multithreaded-v1/main.cpp
METHOD5_MT_OUT = method-5-multithreaded-v1/exe.out

all: method-1 method-2 method-3 method-3-multithreaded-v1 method-4 method-5 method-5-multithreaded-v1

method-1: $(METHOD1_OUT)
$(METHOD1_OUT): $(METHOD1_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

method-2: $(METHOD2_OUT)
$(METHOD2_OUT): $(METHOD2_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

method-3: $(METHOD3_OUT)
$(METHOD3_OUT): $(METHOD3_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

method-3-multithreaded-v1: $(METHOD3_MT_OUT)
$(METHOD3_MT_OUT): $(METHOD3_MT_SRC)
	$(CXX) $(OMPFLAGS) $< -o $@

method-4: $(METHOD4_OUT)
$(METHOD4_OUT): $(METHOD4_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

method-5: $(METHOD5_OUT)
$(METHOD5_OUT): $(METHOD5_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

method-5-multithreaded-v1: $(METHOD5_MT_OUT)
$(METHOD5_MT_OUT): $(METHOD5_MT_SRC)
	$(CXX) $(OMPFLAGS) $< -o $@

run-method-1: method-1
	./$(METHOD1_OUT)

run-method-2: method-2
	./$(METHOD2_OUT)

run-method-3: method-3
	./$(METHOD3_OUT)

run-method-3-multithreaded: method-3-multithreaded-v1
	./$(METHOD3_MT_OUT)

run-method-4: method-4
	./$(METHOD4_OUT)

run-method-5: method-5
	./$(METHOD5_OUT)

run-method-5-multithreaded: method-5-multithreaded-v1
	./$(METHOD5_MT_OUT)

clean:
	rm -f $(METHOD1_OUT) $(METHOD2_OUT) $(METHOD3_OUT) \
	      $(METHOD3_MT_OUT) $(METHOD4_OUT) $(METHOD5_OUT) \
	      $(METHOD5_MT_OUT)