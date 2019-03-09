W        = -Wall
OPT      = -O2 -g
STD      = -std=c++17
CXXFLAGS = $(STD) $(OPT) $(W) -fPIC $(XCXXFLAGS)
INCS     = -Iinclude -Iminiserial -Ihoytech-cpp

LDLIBS   = -lpthread -ldocopt -lb2
LDFLAGS  = -flto $(XLDFLAGS)

SRCS     = ephemerand.cpp util.cpp \
           miniserial/serial.cpp hoytech-cpp/timer.cpp \
           cmd_run.cpp cmd_reset.cpp

BIN      = ephemerand



OBJS    := $(SRCS:.cpp=.o)
DEPS    := $(SRCS:.cpp=.d)

$(BIN): $(OBJS) $(DEPS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $(BIN)

%.o : %.cpp %.d Makefile
	$(CXX) $(CXXFLAGS) $(INCS) -MMD -MP -MT $@ -MF $*.d -c $< -o $@

-include *.d

%.d : ;

.PHONY: clean asan
clean:
	rm -f $(BIN) *.o *.d miniserial/*.o miniserial/*.d hoytech-cpp/*.o hoytech-cpp/*.d
	rm -rf build/

asan: XCXXFLAGS = -fsanitize=address
asan: XLDFLAGS = -fsanitize=address
asan: $(BIN)
