TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += ASICamera2.h

SDK_VERSION = 1.41

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/ASICamera2.lib
BIN_INSTALLS_WIN32    += ../os/windows-x64/ASICamera2.dll

else ifeq ($(OS_CLASS), Linux)
	ifeq ($(ARCH_CLASS), x86_64)
  		LIB_INSTALLS_Linux += ../os/linux-x86_64/libASICamera2.so
  		LIB_INSTALLS_Linux += ../os/linux-x86_64/libASICamera2.so.$(SDK_VERSION)
  	else ifeq ($(ARCH_CLASS), aarch64)
  		LIB_INSTALLS_Linux += ../os/linux-aarch64/libASICamera2.so
  		LIB_INSTALLS_Linux += ../os/linux-aarch64/libASICamera2.so.$(SDK_VERSION)
  	endif

else ifeq ($(OS_CLASS), Darwin)
	ifeq (darwin-x86, $(findstring darwin-x86, $(T_A)))
  		LIB_INSTALLS_Darwin += ../os/darwin-x86/libASICamera2.dylib
  		LIB_INSTALLS_Darwin += ../os/darwin-x86/libASICamera2.dylib.$(SDK_VERSION)
  	else ifeq (darwin-aarch64, $(findstring darwin-aarch64, $(T_A)))
  		LIB_INSTALLS_Darwin += ../os/darwin-aarch64/libASICamera2.dylib
  		LIB_INSTALLS_Darwin += ../os/darwin-aarch64/libASICamera2.dylib.$(SDK_VERSION)
  	endif
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
