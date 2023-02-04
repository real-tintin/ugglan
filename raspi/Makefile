# Makefile for compilation of the drone C++ source
# code for target - a Raspberry Pi Zero 2 W.
#   TARGET_NAME    : Set target name.
#
# Optional
#   BUILD_FOR_TEST : Set to TRUE, to build for testing.
#   GDB_DEBUG      : Set to TRUE, to build for gdb debugging (disables optimization).
#   EXTRA_CXXFLAGS : Use to add extra CXXFLAGS.
#   BUILD_SUFFIX   : Suffix the build dir name.
#

TARGET := $(TARGET_NAME)
SRCEXT := cpp

CXX = g++-10
CXXFLAGS += -Wall -Werror -Wno-psabi -std=c++17 -pthread -MD $(EXTRA_CXXFLAGS)
LDFLAGS += -lz -li2c -lzmq

SRCDIR := ./src
OBJDIR := ./build_$(BUILD_SUFFIX)
INCDIR := ./inc
LIBDIR := ./lib
TESTDIR := ./tests

SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
INC := -I $(INCDIR) -I $(LIBDIR)

ifeq ($(GDB_DEBUG), TRUE)
	CXXFLAGS += -ggdb
else
	CXXFLAGS += -O3
endif

ifeq ($(BUILD_FOR_TEST), TRUE)
	CXXFLAGS += -D UNIT_TEST

	SOURCES := $(filter-out $(SRCDIR)/main.cpp, $(SOURCES))
	SOURCES += $(wildcard $(TESTDIR)/*.$(SRCEXT))

	INC += -I $(TESTDIR)
endif

OBJECTS := $(SOURCES:.$(SRCEXT)=.o)
OBJECTS := $(addprefix $(OBJDIR),$(OBJECTS:.%=%))

DEPENDENCIES := $(OBJECTS:.o=.d)

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
	@echo "Cleaning..."
	$(RM) -r $(OBJDIR) $(TARGET)

-include $(wildcard $(DEPENDENCIES))
