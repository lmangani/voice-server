CC=g++-mp-4.6 
CPPFLAGS=-std=c++0x -I/opt/local/include -Wall -Werror -O4

vserver: media.o g711.o main.o
	$(CC) media.o g711.o main.o ilbc/libilbc.a -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o vserver

media.o: src/media.cpp src/media.hpp
	$(CC) -c $(CPPFLAGS) src/media.cpp -o media.o

main.o: src/main.cpp src/json.hpp
	$(CC) -c $(CPPFLAGS) src/main.cpp -o main.o

g711.o: src/g711.cpp
	$(CC) -c $(CPPFLAGS) src/g711.cpp -o g711.o

json-test: src/json-test.cpp src/json.hpp
	$(CC) $(CPPFLAGS) src/json-test.cpp -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o json-test

clean: 
	rm -f *.o

