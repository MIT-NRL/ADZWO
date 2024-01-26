errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/prosilicaApp.dbd")

prosilicaApp_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("PREFIX", "13PS1:")
epicsEnvSet("PORT",   "PS1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "1360")
epicsEnvSet("YSIZE",  "1024")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

zwoconfig("$(PORT)", 5000698, 50, 0)

asynSetTraceIOMask("$(PORT)", 0, 2)


NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)


< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ZWO)/zwoApp/Db")

iocInit()

create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")
