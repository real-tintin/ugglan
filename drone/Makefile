# Makefile for compilation of the drone C++ source
# code.
#   TARGET_NAME    : Set target name.
#   BUILD_FOR_TEST : Set to TRUE, to build for testing.
#

TARGET := $(TARGET_NAME)
SRCEXT := cpp

CXX = g++
CXXFLAGS += -Wall -std=c++17 -pthread
LDFLAGS += -lz

SRCDIR := ./src
OBJDIR := ./build
INCDIR := ./inc
TESTDIR := ./tests

SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
INC := -I $(INCDIR)

ifeq ($(BUILD_FOR_TEST), TRUE)
	CXXFLAGS += -D UNIT_TEST

	SOURCES += $(wildcard $(TESTDIR)/*.$(SRCEXT))
	INC += -I $(TESTDIR)
endif

OBJECTS := $(SOURCES:.$(SRCEXT)=.o)
OBJECTS := $(addprefix $(OBJDIR),$(OBJECTS:.%=%))

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@echo "Linking..."
	$(LINK.cc) $^ -o $(OBJDIR)/$(TARGET) $(LIB) $(LDFLAGS)

$(OBJECTS): $(OBJDIR)/%.o: %.$(SRCEXT)
	@echo "Compiling..."
	@mkdir -p $(@D)
	$(LINK.cc) $(INC) -c -o $@ $<

clean:
	@echo "Cleaning...";
	$(RM) -r $(OBJDIR) $(TARGET)
