# Default compiler for non-OpenMP targets
CXX = g++
CXXFLAGS = -O3 -std=c++17 -I../include

# OpenMP compiler (Homebrew GCC)
GCC_OMP = /opt/homebrew/bin/g++-15
OMPFLAGS = -O3 -std=c++17 -fopenmp -I../include

# Method 1
METHOD1_SRC = method-1/main.cpp
METHOD1_OUT = method-1/exe.out

# Method 2
METHOD2_SRC = method-2/main.cpp
METHOD2_OUT = method-2/exe.out

# Method 3
METHOD3_SRC = method-3/main.cpp
METHOD3_OUT = method-3/exe.out

# Method 3 (Multithreaded)
METHOD3_MT_SRC = method-3-multithreaded/main.cpp
METHOD3_MT_OUT = method-3-multithreaded/exe.out

# Method 4
METHOD4_SRC = method-4/main.cpp
METHOD4_OUT = method-4/exe.out

# Method 5
METHOD5_SRC = method-5/main.cpp
METHOD5_OUT = method-5/exe.out

# Default: build all methods
all: method-1 method-2 method-3 method-3-multithreaded method-4 method-5

# Method 1
method-1: $(METHOD1_OUT)
$(METHOD1_OUT): $(METHOD1_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Method 2
method-2: $(METHOD2_OUT)
$(METHOD2_OUT): $(METHOD2_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Method 3
method-3: $(METHOD3_OUT)
$(METHOD3_OUT): $(METHOD3_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Method 3 (Multithreaded with OpenMP)
method-3-multithreaded: $(METHOD3_MT_OUT)
$(METHOD3_MT_OUT): $(METHOD3_MT_SRC)
	$(GCC_OMP) $(OMPFLAGS) $< -o $@

# Method 4
method-4: $(METHOD4_OUT)
$(METHOD4_OUT): $(METHOD4_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Method 5
method-5: $(METHOD5_OUT)
$(METHOD5_OUT): $(METHOD5_SRC)
	$(CXX) $(CXXFLAGS) $< -o $@

# Run targets
run-method-1: method-1
	./$(METHOD1_OUT)

run-method-2: method-2
	./$(METHOD2_OUT)

run-method-3: method-3
	./$(METHOD3_OUT)

run-method-3-multithreaded: method-3-multithreaded
	./$(METHOD3_MT_OUT)

run-method-4: method-4
	./$(METHOD4_OUT)

run-method-5: method-5
	./$(METHOD5_OUT)

# Clean targets
clean:
	rm -f $(METHOD1_OUT) $(METHOD2_OUT) $(METHOD3_OUT) \
	      $(METHOD3_MT_OUT) $(METHOD4_OUT) $(METHOD5_OUT)

