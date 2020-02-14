# Makefile for compilation of catch2
# C++ unit tests.
#
# Note, the project dependencies needs to
# be compiled before. Use run_catch_tests.sh.
#

TARGET := $(TARGET_NAME)

CXX = g++
CXXFLAGS += -Wall -std=c++11

SRCDIR := .
BUILDDIR := ../build
LIBDIR := $(BUILDDIR)
INCLUDEDIR := ../include

SRCEXT := cpp
LIBEXT := o

SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
LIB := $(wildcard $(LIBDIR)/*.$(LIBEXT))
INC := -I $(INCLUDEDIR)

$(TARGET): $(OBJECTS)
	@echo "Linking catch unit-tests..."
	@echo " $(LINK.cc) $^ -o $(TARGET) $(LIB)"; $(LINK.cc) $^ -o $(TARGET) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@echo "Compiling catch unit-tests..."
	@mkdir -p $(BUILDDIR)
	@echo " $(LINK.cc) $(INC) -c -o $@ $<"; $(LINK.cc) $(INC) -c -o $@ $<
