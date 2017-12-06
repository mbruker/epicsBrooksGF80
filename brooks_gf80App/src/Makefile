TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += brooks_gf80

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install brooks_gf80.dbd into <top>/dbd
DBD += brooks_gf80.dbd

# specify all source files to be compiled and added to the library
brooks_gf80_SRCS += brooks_gf80_driver.cpp

brooks_gf80_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

