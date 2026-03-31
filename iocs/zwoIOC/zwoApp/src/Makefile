TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = ZWODriverApp
PROD_IOC  += $(PROD_NAME)

# Keep this IOC minimal so it can boot as a simple standalone test app.
WITH_QSRV = NO
WITH_PVA  = NO

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ZWODriverSupport.dbd
$(PROD_NAME)_DBD += busySupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS_DEFAULT += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
PROD_SRCS_vxWorks += $(PROD_NAME)_registerRecordDeviceDriver.cpp

PROD_LIBS += ZWODriver
PROD_LIBS += busy

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
