

SHELL = /bin/sh

.SUFFIXES:
.SUFFIXES: .c .cpp .o .y .l .tab.h .tab.c .tab.o 
.PHONY: all clean cleanall tar

find_files = $(wildcard $(dir)/*)
find_files_with_extension = $(wildcard $(dir)/*.*)

SOURCE_DIR = src
BIN_DIR = bin
EXE_DIR = robot
CONFIG  = config
BEHAVIORS = behaviors
LOG = log

ALL_SOURCE_DIRS = $(wildcard */src)
ALL_INCLUDE_DIRS = $(wildcard */include)
ALL_LIB_DIRS = $(wildcard */lib)
ALL_LIB_BINS = $(patsubst %lib, %bin, $(ALL_LIB_DIRS))
ALL_ROBOT_DIRS = $(notdir $(filter-out %CVS, $(wildcard robot/config/*)))
ALL_CONFIG_DIRS = $(shell find $(EXE_DIR) -name *config*)
ALL_UTILS_DIRS = $(dir $(filter-out robot%, $(wildcard */src/*/Makefile)))


vpath %.h $(ALL_INCLUDE_DIRS)

ifdef MODULE

MODULE_UTILS = $(filter $(MODULE)%, $(ALL_UTILS_DIRS))

ifneq (,$(findstring $(MODULE),$(ALL_LIB_DIRS)))
all: FORCE
	cd $(MODULE)/$(SOURCE_DIR); make lib
#ifneq (,$(MODULE_UTILS))
#	for dir in $(MODULE_UTILS); do	\
#		cd $$dir; make ROBOT=$(ROBOT) exe; cd ../../..;	\
#	done
#endif

test: 
	cd $(MODULE)/$(SOURCE_DIR); make test 

doc: 
	cd $(MODULE)/$(SOURCE_DIR); make doc

clean:
	cd $(MODULE)/$(SOURCE_DIR); make clean

cleanall:
	cd $(MODULE)/$(SOURCE_DIR); make cleanall

tar:
	cd $(MODULE)/$(SOURCE_DIR); make tar

else
all: FORCE
	$(error The module $(MODULE) does not exist)
endif

else

ifndef ROBOT
ROBOT=robot
endif

ALL_BIN_FILES = $(filter-out %CVS, $(foreach dir,$(ALL_LIB_BINS),$(find_files)))
ALL_CONFIG_FILES = $(foreach dir,$(ALL_CONFIG_DIRS) $(addsuffix /$(ROBOT), $(ALL_CONFIG_DIRS)), $(find_files_with_extension))

ifeq (,$(findstring $(ROBOT),$(ALL_ROBOT_DIRS)))
all: FORCE
	$(error The robot $(ROBOT) does not exist)
else

exe_name = $(ROBOT)

all: libs exe utils robot_dir copy

libs: $(ALL_LIB_DIRS)

%/lib/: FORCE
	MODULE=$*
	cd $*/$(SOURCE_DIR); make lib

exe: FORCE
	cd $(EXE_DIR)/$(SOURCE_DIR); make exe

utils: FORCE
	for dir in $(ALL_UTILS_DIRS); do	\
		cd $$dir; make ROBOT=$(ROBOT) exe; cd ../../..;	\
	done

robot_dir: FORCE
	mkdir -p $(ROBOT)
	mkdir -p $(ROBOT)/$(CONFIG)
	mkdir -p $(ROBOT)/$(LOG)


copy : 	ALL_BIN_FILES = $(filter-out %CVS, $(foreach dir,$(ALL_LIB_BINS),$(find_files))) 
copy : copy-bin #copy-config

copy-bin : $(EXE_DIR)/$(BIN_DIR)/$(ROBOT) $(ALL_BIN_FILES) 
	-cp -u $^ $(ROBOT)

copy-config : copy-config-others #copy-config-brian

copy-config-brian : $(filter $(EXE_DIR)/brian%, $(ALL_CONFIG_FILES))
	-cp -u $^ $(ROBOT)/$(BEHAVIORS)

copy-config-others : $(filter-out $(EXE_DIR)/brian%, $(ALL_CONFIG_FILES))
	-cp -u $^ $(ROBOT)/$(CONFIG)
endif

monitor: FORCE
	cd monitor/src; make exe_name=monitor exe
	cd monitor/src/monitorgui; make exe
	cp monitor/src/monitor.sh monitor/bin/
	chmod +x monitor/bin/monitor.sh
	cp monitor/config/* monitor/bin/

refbox: FORCE
	cd refbox/src; make exe_name=refbox exe
#	cp refbox/ monitor/bin

clean:
	for dir in $(ALL_SOURCE_DIRS); do	\
		cd $$dir; make clean; cd ../..;	\
	done
	for dir in $(ALL_UTILS_DIRS); do	\
		cd $$dir; make clean; cd ../../..;	\
	done
#	-rm -fr $(ALL_ROBOT_DIRS)

cleanall:
	-find . -name *.d | xargs rm
	for dir in $(ALL_SOURCE_DIRS); do	\
		cd $$dir; make cleanall; cd ../..;	\
	done
	for dir in $(ALL_UTILS_DIRS); do	\
		cd $$dir; make cleanall; cd ../../..;	\
	done
	-rm -fr $(ALL_ROBOT_DIRS)

tar: cleanall
	cd ../..;tar cvfz MRT.tgz MRT

endif


FORCE: ;
