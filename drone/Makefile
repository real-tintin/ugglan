# Makefile for compilation of the drone C++ source
# code.
#   TARGET_NAME    : Set target name.
# 	BUILD_FOR_TEST : Set to TRUE, to build for testing.
#

TARGET := $(TARGET_NAME)
SRCEXT := cpp

CXX = g++
CXXFLAGS += -Wall -std=c++11

SRCDIR := ./src
BUILDDIR := ./build
INCLUDEDIR := ./include
TESTDIR = ./tests

SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))

LIB :=
INC := -I $(INCLUDEDIR)

ifeq ($(BUILD_FOR_TEST), TRUE)
	CXXFLAGS += -D UNIT_TEST

	TESTSOURCES += $(wildcard $(TESTDIR)/*.$(SRCEXT))
	OBJECTS += $(patsubst $(TESTDIR)/%,$(BUILDDIR)/%,$(TESTSOURCES:.$(SRCEXT)=.o))

	INC += -I $(TESTDIR)
endif

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@echo "Linking..."
	$(LINK.cc) $^ -o $(BUILDDIR)/$(TARGET) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@echo "Compiling src..."
	@mkdir -p $(BUILDDIR)
	$(LINK.cc) $(INC) -c -o $@ $<

$(BUILDDIR)/%.o: $(TESTDIR)/%.$(SRCEXT)
	@echo "Compiling tests..."
	@mkdir -p $(BUILDDIR)
	$(LINK.cc) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	$(RM) -r $(BUILDDIR) $(TARGET)
