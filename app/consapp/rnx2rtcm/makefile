# makefile for rnx2rtcm

BINDIR = ../../../bin
SRC    = ../../../src
OPTIONS= -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=4 -DNEXOBS=3
CFLAGS = -std=c99 -Wall -O3 -pedantic -I$(SRC) $(OPTIONS)
LDLIBS  = -lm

rnx2rtcm   : rnx2rtcm.o trace.o rtkcmn.o rinex.o rtcm.o rtcm2.o rtcm3.o rtcm3e.o

rnx2rtcm.o : rnx2rtcm.c
	$(CC) -c $(CFLAGS) rnx2rtcm.c
rtkcmn.o   : $(SRC)/rtkcmn.c
	$(CC) -c $(CFLAGS) $(SRC)/rtkcmn.c
trace.o   : $(SRC)/rtklib.h $(SRC)/trace.c
	$(CC) -c $(CFLAGS) $(SRC)/trace.c
rinex.o    : $(SRC)/rinex.c
	$(CC) -c $(CFLAGS) $(SRC)/rinex.c
rtcm.o     : $(SRC)/rtcm.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm.c
rtcm2.o    : $(SRC)/rtcm2.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm2.c
rtcm3.o    : $(SRC)/rtcm3.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3.c
rtcm3e.o   : $(SRC)/rtcm3e.c
	$(CC) -c $(CFLAGS) $(SRC)/rtcm3e.c

install:
	cp rnx2rtcm $(BINDIR)

clean:
	rm -f rnx2rtcm rnx2rtcm.exe *.o