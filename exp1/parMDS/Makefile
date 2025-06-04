all: parMDS seqMDS

parMDS: parMDS.cpp
    # clang++ -O3 -std=c++14 parMDS.cpp -o parMDS.out
	# nvc++ -O3 -std=c++14 -acc=multicore  parMDS.cpp -o parMDS.out && ./parMDS.out toy.vrp -nthreads 20 -round 1
	
seqMDS: seqMDS.cpp
	g++ -O3 -std=c++14 seqMDS.cpp -o seqMDS.out && ./seqMDS.out toy.vrp
	g++ -O3 -std=c++14 seqMDS_bfs.cpp -o seqMDS_bfs.out && ./seqMDS_bfs.out toy.vrp

clean:
	rm -f *.out

cleanFolders:
	rm -rf outinputs*
