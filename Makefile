TARGET = i2cbridge
BINDIR = /usr/local/sbin/
#INITDIR = /etc/init.d/
#INITEXT = .init
#INITFILE = $(TARGET)$(INITEXT)
CFGDIR = /etc/
CFGEXT = .cfg
CFGFILE = $(TARGET)$(CFGEXT)

SERVICE = $(TARGET).service
SERVICEDIR = /etc/systemd/system

# - Compiler
CC=gcc
CXX=g++
CFLAGS = -g -Wall -Wno-unused -Wno-unknown-pragmas

# - Linker
LIBS = -lwiringPi -lwiringPiDev -lpthread -lstdc++ -lm -lmosquitto -lconfig++

OBJDIR = ./obj
HWDIR = hardware/

.PHONY: default all celan

all: default

#VPATH = aprs

CSRCS += $(wildcard *.c)
CSRCS += $(wildcard $(HWDIR)*.c)
CPPSRCS += $(wildcard *.cpp)
CPPSRCS += $(wildcard $(HWDIR)*.cpp)

COBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(CSRCS))
CPPOBJS = $(patsubst %.cpp,$(OBJDIR)/%.o,$(CPPSRCS))

SRCS = $(CSRCS) $(CPPSRCS)
OBJS = $(COBJS) $(CPPOBJS)

#SRC = $(wildcard *.cpp) $(wildcard aprs/*.c*)
#HDR = $(wildcard *.h) $(wildcard aprs/*.h)
#OBJ = $(SRC:%.c=%.o)

#OBJECTS = $(patsubst %c, %o, $(wildcard *.c))
#HEADERS = $(wildcard *.h)

$(OBJDIR)/%.o: %.c
	@mkdir -p $(OBJDIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(OBJDIR)
	@echo "CXX $<"
	@$(CXX) $(CFLAGS) -c $< -o $@

$(OBJDIR)/$(HWDIR)vimon.o: $(HWDIR)vimon_cal.h $(HWDIR)vimon.h
$(OBJDIR)/$(HWDIR)MCP9808.o: $(HWDIR)MCP9808.h
$(OBJDIR)/modbustag.o: modbustag.h
$(OBJDIR)/hardware.o: hardware.h
$(OBJDIR)/datatag.o: datatag.h
$(OBJDIR)/mqtt.o: mqtt.h
$(OBJDIR)/i2cbridge.o: i2cbridge.h

default: $(OBJS)
	$(CC) $(OBJS) $(MODULES) $(LDFLAGS) $(LIBS) -o $(TARGET)

#$(TARGET): $(OBJ)
#	make -C $(MODULEDIR)
#	$(CC) $(OBJ) $(MODULES) -Wall $(LIBS) -o $@
#	$(CC) $(OBJ) -Wall $(LIBS) -o $@

#%.o: %.c $(HDR)
#	@echo $(SRC)
#	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJ)

#
# install target and config file
#
install:
ifneq ($(shell id -u), 0)
	@echo "!!!! install requires root !!!!"
else
	install -o root $(TARGET) $(BINDIR)$(TARGET)
	install -o root $(CFGFILE) $(CFGDIR)$(CFGFILE)
#	install -m 755 -o root $(INITFILE) $(INITDIR)$(INITFILE)
#	mv $(INITDIR)$(INITFILE) $(INITDIR)$(TARGET)
	@echo ++++++++++++++++++++++++++++++++++++++++++++
	@echo ++ Files have been installed
	@echo ++ You will need to restart $(TARGET)
	@echo ++ sudo systemctl restart $(TARGET)
endif

#
# make systemd service
#
service:
ifneq ($(shell id -u), 0)
	@echo "!!!! service requires root !!!!"
else
	install -o root $(SERVICE) $(SERVICEDIR)
	@systemctl daemon-reload
	@systemctl enable $(SERVICE)
	@echo $(TARGET) is now available as systemd service
endif

clean:
	-rm -f *.o
	-rm -f $(OBJS)
#	-rm -rf $(TARGET)

.PHONY : clean install
