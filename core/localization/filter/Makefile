CCFLAGS=-std=c++14 -O2 -I/home/prateeti/Dropbox/sem1/CS393R/project
CC=g++
headers = $(wildcard *.hh)
codes = $(wildcard *.cc)

filter : $(codes) $(headers)
	$(CC) $(CCFLAGS) -o filter $(codes)

.PHONY: clean

clean:
	rm -f filter

# filter : main.cc filter.cc filter.hh motion_update.cc sensor_update.cc io.cc\
# 			ParticleFilter.cc ParticleFilter.hh\
# 			GridFilter.cc GridFilter.hh
# 	$(CC) $(CCFLAGS) -o filter main.cc filter.cc motion_update.cc sensor_update.cc io.cc\
# 			ParticleFilter.cc GridFilter.cc

# filter: dtec.o model.o parser.o
# 	$(CC) -o dtec dtec.o model.o parser.o

# dtec.o: dtec.cc model.hh
# 	$(CC) $(CCFLAGS) -c dtec.cc

# model.o: model.cc model.hh
# 	$(CC) $(CCFLAGS) -c model.cc

# parser.o: parser.cc model.hh
# 	$(CC) $(CCFLAGS) -c parser.cc

# .PHONY: clean

# clean:
# 	rm -f *.o *~ dtec


# CPPFLAGS=-std=c++14
# SDIR=src
# CC=g++

# dtec: dtec.o model.o
# 	$(CC) -o dtec dtec.o model.o 

# dtec.o: $(SDIR)/dtec.cc
# 	$(CC) $(CPPFLAGS) -c dtec.cc

# model.o: $(SDIR)/model.cc $(SDIR)/model.hh
# 	$(CC) $(CPPFLAGS) -c model.cc