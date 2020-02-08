#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../nmea/context.c ../nmea/generate.c ../nmea/generator.c ../nmea/gmath.c ../nmea/info.c ../nmea/parse.c ../nmea/parser.c ../nmea/sentence.c ../nmea/time.c ../nmea/tok.c ../main.c ../sensors.c ../coordinates.c ../navigation_helper.c ../delay.c ../servo.c ../tft_gfx.c ../tft_master.c ../test.c ../PID.c ../radio.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/760772214/context.o ${OBJECTDIR}/_ext/760772214/generate.o ${OBJECTDIR}/_ext/760772214/generator.o ${OBJECTDIR}/_ext/760772214/gmath.o ${OBJECTDIR}/_ext/760772214/info.o ${OBJECTDIR}/_ext/760772214/parse.o ${OBJECTDIR}/_ext/760772214/parser.o ${OBJECTDIR}/_ext/760772214/sentence.o ${OBJECTDIR}/_ext/760772214/time.o ${OBJECTDIR}/_ext/760772214/tok.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/sensors.o ${OBJECTDIR}/_ext/1472/coordinates.o ${OBJECTDIR}/_ext/1472/navigation_helper.o ${OBJECTDIR}/_ext/1472/delay.o ${OBJECTDIR}/_ext/1472/servo.o ${OBJECTDIR}/_ext/1472/tft_gfx.o ${OBJECTDIR}/_ext/1472/tft_master.o ${OBJECTDIR}/_ext/1472/test.o ${OBJECTDIR}/_ext/1472/PID.o ${OBJECTDIR}/_ext/1472/radio.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/760772214/context.o.d ${OBJECTDIR}/_ext/760772214/generate.o.d ${OBJECTDIR}/_ext/760772214/generator.o.d ${OBJECTDIR}/_ext/760772214/gmath.o.d ${OBJECTDIR}/_ext/760772214/info.o.d ${OBJECTDIR}/_ext/760772214/parse.o.d ${OBJECTDIR}/_ext/760772214/parser.o.d ${OBJECTDIR}/_ext/760772214/sentence.o.d ${OBJECTDIR}/_ext/760772214/time.o.d ${OBJECTDIR}/_ext/760772214/tok.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/sensors.o.d ${OBJECTDIR}/_ext/1472/coordinates.o.d ${OBJECTDIR}/_ext/1472/navigation_helper.o.d ${OBJECTDIR}/_ext/1472/delay.o.d ${OBJECTDIR}/_ext/1472/servo.o.d ${OBJECTDIR}/_ext/1472/tft_gfx.o.d ${OBJECTDIR}/_ext/1472/tft_master.o.d ${OBJECTDIR}/_ext/1472/test.o.d ${OBJECTDIR}/_ext/1472/PID.o.d ${OBJECTDIR}/_ext/1472/radio.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/760772214/context.o ${OBJECTDIR}/_ext/760772214/generate.o ${OBJECTDIR}/_ext/760772214/generator.o ${OBJECTDIR}/_ext/760772214/gmath.o ${OBJECTDIR}/_ext/760772214/info.o ${OBJECTDIR}/_ext/760772214/parse.o ${OBJECTDIR}/_ext/760772214/parser.o ${OBJECTDIR}/_ext/760772214/sentence.o ${OBJECTDIR}/_ext/760772214/time.o ${OBJECTDIR}/_ext/760772214/tok.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/sensors.o ${OBJECTDIR}/_ext/1472/coordinates.o ${OBJECTDIR}/_ext/1472/navigation_helper.o ${OBJECTDIR}/_ext/1472/delay.o ${OBJECTDIR}/_ext/1472/servo.o ${OBJECTDIR}/_ext/1472/tft_gfx.o ${OBJECTDIR}/_ext/1472/tft_master.o ${OBJECTDIR}/_ext/1472/test.o ${OBJECTDIR}/_ext/1472/PID.o ${OBJECTDIR}/_ext/1472/radio.o

# Source Files
SOURCEFILES=../nmea/context.c ../nmea/generate.c ../nmea/generator.c ../nmea/gmath.c ../nmea/info.c ../nmea/parse.c ../nmea/parser.c ../nmea/sentence.c ../nmea/time.c ../nmea/tok.c ../main.c ../sensors.c ../coordinates.c ../navigation_helper.c ../delay.c ../servo.c ../tft_gfx.c ../tft_master.c ../test.c ../PID.c ../radio.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX250F128B
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/760772214/context.o: ../nmea/context.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/context.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/context.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/context.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/context.o.d" -o ${OBJECTDIR}/_ext/760772214/context.o ../nmea/context.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/generate.o: ../nmea/generate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/generate.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/generate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/generate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/generate.o.d" -o ${OBJECTDIR}/_ext/760772214/generate.o ../nmea/generate.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/generator.o: ../nmea/generator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/generator.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/generator.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/generator.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/generator.o.d" -o ${OBJECTDIR}/_ext/760772214/generator.o ../nmea/generator.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/gmath.o: ../nmea/gmath.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/gmath.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/gmath.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/gmath.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/gmath.o.d" -o ${OBJECTDIR}/_ext/760772214/gmath.o ../nmea/gmath.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/info.o: ../nmea/info.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/info.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/info.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/info.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/info.o.d" -o ${OBJECTDIR}/_ext/760772214/info.o ../nmea/info.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/parse.o: ../nmea/parse.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/parse.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/parse.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/parse.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/parse.o.d" -o ${OBJECTDIR}/_ext/760772214/parse.o ../nmea/parse.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/parser.o: ../nmea/parser.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/parser.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/parser.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/parser.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/parser.o.d" -o ${OBJECTDIR}/_ext/760772214/parser.o ../nmea/parser.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/sentence.o: ../nmea/sentence.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/sentence.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/sentence.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/sentence.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/sentence.o.d" -o ${OBJECTDIR}/_ext/760772214/sentence.o ../nmea/sentence.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/time.o: ../nmea/time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/time.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/time.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/time.o.d" -o ${OBJECTDIR}/_ext/760772214/time.o ../nmea/time.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/tok.o: ../nmea/tok.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/tok.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/tok.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/tok.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/tok.o.d" -o ${OBJECTDIR}/_ext/760772214/tok.o ../nmea/tok.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/sensors.o: ../sensors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/sensors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/sensors.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/sensors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/sensors.o.d" -o ${OBJECTDIR}/_ext/1472/sensors.o ../sensors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/coordinates.o: ../coordinates.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/coordinates.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/coordinates.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/coordinates.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/coordinates.o.d" -o ${OBJECTDIR}/_ext/1472/coordinates.o ../coordinates.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/navigation_helper.o: ../navigation_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/navigation_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/navigation_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/navigation_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/navigation_helper.o.d" -o ${OBJECTDIR}/_ext/1472/navigation_helper.o ../navigation_helper.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/delay.o: ../delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/delay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/delay.o.d" -o ${OBJECTDIR}/_ext/1472/delay.o ../delay.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/servo.o: ../servo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/servo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/servo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/servo.o.d" -o ${OBJECTDIR}/_ext/1472/servo.o ../servo.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/tft_gfx.o: ../tft_gfx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_gfx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_gfx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/tft_gfx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/tft_gfx.o.d" -o ${OBJECTDIR}/_ext/1472/tft_gfx.o ../tft_gfx.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/tft_master.o: ../tft_master.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_master.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/tft_master.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/tft_master.o.d" -o ${OBJECTDIR}/_ext/1472/tft_master.o ../tft_master.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/test.o: ../test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/test.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/test.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/test.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/test.o.d" -o ${OBJECTDIR}/_ext/1472/test.o ../test.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/PID.o: ../PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/PID.o.d" -o ${OBJECTDIR}/_ext/1472/PID.o ../PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/radio.o: ../radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/radio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/radio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/radio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/radio.o.d" -o ${OBJECTDIR}/_ext/1472/radio.o ../radio.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
else
${OBJECTDIR}/_ext/760772214/context.o: ../nmea/context.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/context.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/context.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/context.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/context.o.d" -o ${OBJECTDIR}/_ext/760772214/context.o ../nmea/context.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/generate.o: ../nmea/generate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/generate.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/generate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/generate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/generate.o.d" -o ${OBJECTDIR}/_ext/760772214/generate.o ../nmea/generate.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/generator.o: ../nmea/generator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/generator.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/generator.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/generator.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/generator.o.d" -o ${OBJECTDIR}/_ext/760772214/generator.o ../nmea/generator.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/gmath.o: ../nmea/gmath.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/gmath.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/gmath.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/gmath.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/gmath.o.d" -o ${OBJECTDIR}/_ext/760772214/gmath.o ../nmea/gmath.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/info.o: ../nmea/info.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/info.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/info.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/info.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/info.o.d" -o ${OBJECTDIR}/_ext/760772214/info.o ../nmea/info.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/parse.o: ../nmea/parse.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/parse.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/parse.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/parse.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/parse.o.d" -o ${OBJECTDIR}/_ext/760772214/parse.o ../nmea/parse.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/parser.o: ../nmea/parser.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/parser.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/parser.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/parser.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/parser.o.d" -o ${OBJECTDIR}/_ext/760772214/parser.o ../nmea/parser.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/sentence.o: ../nmea/sentence.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/sentence.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/sentence.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/sentence.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/sentence.o.d" -o ${OBJECTDIR}/_ext/760772214/sentence.o ../nmea/sentence.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/time.o: ../nmea/time.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/time.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/time.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/time.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/time.o.d" -o ${OBJECTDIR}/_ext/760772214/time.o ../nmea/time.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/760772214/tok.o: ../nmea/tok.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/760772214" 
	@${RM} ${OBJECTDIR}/_ext/760772214/tok.o.d 
	@${RM} ${OBJECTDIR}/_ext/760772214/tok.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/760772214/tok.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/760772214/tok.o.d" -o ${OBJECTDIR}/_ext/760772214/tok.o ../nmea/tok.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/sensors.o: ../sensors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/sensors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/sensors.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/sensors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/sensors.o.d" -o ${OBJECTDIR}/_ext/1472/sensors.o ../sensors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/coordinates.o: ../coordinates.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/coordinates.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/coordinates.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/coordinates.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/coordinates.o.d" -o ${OBJECTDIR}/_ext/1472/coordinates.o ../coordinates.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/navigation_helper.o: ../navigation_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/navigation_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/navigation_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/navigation_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/navigation_helper.o.d" -o ${OBJECTDIR}/_ext/1472/navigation_helper.o ../navigation_helper.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/delay.o: ../delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/delay.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/delay.o.d" -o ${OBJECTDIR}/_ext/1472/delay.o ../delay.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/servo.o: ../servo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/servo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/servo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/servo.o.d" -o ${OBJECTDIR}/_ext/1472/servo.o ../servo.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/tft_gfx.o: ../tft_gfx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_gfx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_gfx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/tft_gfx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/tft_gfx.o.d" -o ${OBJECTDIR}/_ext/1472/tft_gfx.o ../tft_gfx.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/tft_master.o: ../tft_master.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/tft_master.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/tft_master.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/tft_master.o.d" -o ${OBJECTDIR}/_ext/1472/tft_master.o ../tft_master.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/test.o: ../test.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/test.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/test.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/test.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/test.o.d" -o ${OBJECTDIR}/_ext/1472/test.o ../test.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/PID.o: ../PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/PID.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/PID.o.d" -o ${OBJECTDIR}/_ext/1472/PID.o ../PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1472/radio.o: ../radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/radio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/radio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/radio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -O1 -MMD -MF "${OBJECTDIR}/_ext/1472/radio.o.d" -o ${OBJECTDIR}/_ext/1472/radio.o ../radio.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1  -mdfp=${DFP_DIR}
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -O1  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC00490:0x1FC00BEF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=800,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp=${DFP_DIR}
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -O1  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=800,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp=${DFP_DIR}
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/NavAlgoV1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
