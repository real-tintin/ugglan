# Makefile for compilation of the drone C++ source
# code.
# 	DO_NOT_LINK 	: Set to TRUE, to skip linking.
#

TARGET := catch

CXX = g++
CXXFLAGS += -Wall -std=c++11

SRCDIR := ./src
BUILDDIR := ./build
INCLUDEDIR := ./include

SRCEXT := cpp
SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
LIB :=
INC := -I $(INCLUDEDIR)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
ifneq ($(DO_NOT_LINK),TRUE)
	@echo "Linking drone project..."
	@echo " $(LINK.cc) $^ -o $(TARGET) $(LIB)"; $(LINK.cc) $^ -o $(TARGET) $(LIB)
endif

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@echo "Compiling drone objectives..."
	@mkdir -p $(BUILDDIR)
	@echo " $(LINK.cc) $(INC) -c -o $@ $<"; $(LINK.cc) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning drone build...";
	@echo " $(RM) -r $(BUILDDIR) $(TARGET)"; $(RM) -r $(BUILDDIR) $(TARGET)
