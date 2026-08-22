# Makefile za RPI_SFSM - kompajlira svaki FSM pattern u poseban izvrsni fajl.
#
# Preduslovi na Raspberry Pi (Raspberry Pi OS / Linux):
#   1) Hardverski cycle-counter (perf_event_open) zahteva dozvolu za citanje
#      performance countera iz user-space-a. Ili pokreni binarne fajlove sa
#      sudo, ili privremeno/trajno otvori pristup:
#        sudo sysctl -w kernel.perf_event_paranoid=-1
#      (za trajno: dodaj "kernel.perf_event_paranoid = -1" u /etc/sysctl.conf)
#   2) g++ mora biti instaliran: sudo apt install build-essential
#
# Upotreba:
#   make                     - build svih patterna u bin/
#   ./bin/fsm_hsm_nested     - pokretanje pojedinacnog testa (komande: 1 0 t b r q)
#   make clean               - brisanje bin/ foldera
#
# Napomena o determinizmu: Linux NIJE realtime OS. pin_to_cpu0() u svakom
# fajlu smanjuje (ne eliminise) sum od migracije procesa izmedju jezgara;
# za jos manji sum mozes pokrenuti sa "sudo chrt -f 99 ./bin/<test>" i/ili
# izolovati jezgro preko "isolcpus=" kernel parametra.

CXX = g++
CXXFLAGS = -O2 -Wall -std=c++11
LDLIBS = -lrt

SOURCES = $(wildcard src/*.cpp)
TARGETS = $(patsubst src/%.cpp,bin/%,$(SOURCES))

all: $(TARGETS)

bin/%: src/%.cpp | bin
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

bin:
	mkdir -p bin

clean:
	rm -rf bin

.PHONY: all clean
