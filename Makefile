CC=g++-mp-4.6 
CPPFLAGS=-std=c++0x -O3 -fdata-sections -ffunction-sections -I/opt/local/include -Wall -Werror -g0 -save-temps=obj

vserver: media.o g711.o main.o
	$(CC) media.o g711.o main.o ilbc/libilbc.a -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o vserver

media.o: src/media.cpp src/media.hpp src/media_core.hpp src/media_transport.hpp src/media_audio.hpp
	$(CC) -c $(CPPFLAGS) src/media.cpp -o media.o

main.o: src/main.cpp src/json.hpp src/media_core.hpp src/media_audio.hpp src/media_transport.hpp
	$(CC) -c $(CPPFLAGS) src/main.cpp -o main.o

g711.o: src/g711.cpp
	$(CC) -c $(CPPFLAGS) src/g711.cpp -o g711.o

json-test: src/json-test.cpp src/json.hpp
	$(CC) $(CPPFLAGS) src/json-test.cpp -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o json-test

play: src/tests/play.cpp media.o g711.o
	$(CC) $(CPPFLAGS) src/tests/play.cpp ilbc/libilbc.a g711.o media.o -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o play

record: src/tests/record.cpp media.o g711.o
	$(CC) $(CPPFLAGS) src/tests/record.cpp ilbc/libilbc.a g711.o media.o -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o record

null: src/tests/null.cpp media.o g711.o
	$(CC) $(CPPFLAGS) src/tests/null.cpp g711.o media.o ilbc/libilbc.a -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -Wl,-dead_strip -o null

push-pull: src/tests/push-pull.cpp media.o g711.o
	$(CC) $(CPPFLAGS) src/tests/push-pull.cpp ilbc/libilbc.a g711.o media.o -L/opt/local/lib -l boost_system-mt -l boost_thread-mt -l boost_date_time -o push-pull

clean: 
	rm -f *.o

