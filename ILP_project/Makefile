
all: sdp_ilp

sdp_ilp: sdp_ilp.o
	$(CCC) $(CCFLAGS) sdp_ilp.o -o sdp_ilp $(CCLNFLAGS)
sdp_ilp.o: sdp_ilp.cpp
	$(CCC) -c $(CCFLAGS) sdp_ilp.cpp -o sdp_ilp.o

# ex1: ex1.o
# 	$(CCC) $(CCFLAGS) ex1.o -o ex1 $(CCLNFLAGS)
# ex1.o: ex1.cpp
# 	$(CCC) -c $(CCFLAGS) ex1.cpp -o ex1.o

# ex2: ex2.o
# 	$(CCC) $(CCFLAGS) ex2.o -o ex2 $(CCLNFLAGS)
# ex2.o: ex2.cpp
# 	$(CCC) -c $(CCFLAGS) ex2.cpp -o ex2.o

clean:
	rm --force sdp_ilp sdp_ilp.o
# 	rm --force ex1 ex1.o ex2 ex2.o

################################

SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic
CPLEXDIR      = /opt/ibm/ILOG/CPLEX_Studio2211/cplex
CONCERTDIR    = /opt/ibm/ILOG/CPLEX_Studio2211/concert

################

CCC = g++

CCOPT = -std=c++11 -m64 -O3 -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread -ldl

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) 
