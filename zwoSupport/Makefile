TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += ASICamera2.h

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/ASICamera2.lib
BIN_INSTALLS_WIN32    += ../os/windows-x64/ASICamera2.ddl

else ifeq ($(OS_CLASS), Linux)
	ifeq ($(ARCH_CLASS), x86_64)
  		LIB_INSTALLS_Linux += ../os/linux-x86_64/libASICamera2.so
  	else ifeq ($(ARCH_CLASS), aarch64)
  		LIB_INSTALLS_Linux += ../os/linux-aarch64/libASICamera2.so
  	endif
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE