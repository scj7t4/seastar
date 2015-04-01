default: all

all: seastar.h seastar.c
	g++ -c -fpic -O3 seastar.c
	g++ -shared -o libseastar.so seastar.o 
