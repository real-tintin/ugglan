# Makefile for compilation of the drone C++ source
# code for target - a Raspberry Pi Zero 2 W.
#   TARGET_NAME    : Set target name.
#   BUILD_FOR_TEST : Set to TRUE, to build for testing.
#   EXTRA_CXXFLAGS : Use to add extra CXXFLAGS.
#

TARGET := $(TARGET_NAME)
SRCEXT := cpp

CXX = g++-10
CXXFLAGS += -Wall -Wno-psabi -std=c++17 -pthread -O3 $(EXTRA_CXXFLAGS)
LDFLAGS += -lz -li2c

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
