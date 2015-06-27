#!/bin/bash
################################################################################
#
# Build the EPCS bootloader for a given board design (board.settings)
#
# Copyright (c) 2015, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

SWINFO_FILE=bootloader.swinfo
SETTINGS_FILE=bootloader.settings
HEX_FILE=epcsboot.hex

# process arguments
BOARD_PATH=$1

if [ $1 == "--help" ];
then
    echo "Usage: build_bootloader.sh [BOARD]"
    echo
    echo "BOARD ....................... Path to hardware project (board.settings)"
    echo
    echo "Builds EPCS bootloader by referencing to the provided board.settings file."
    echo "After sourcing all the needed information from board.settings it runs"
    echo "the makefile to generate a hex file for the EPCS ipcore. Then it replaces"
    echo "the hex file in the Quartus project."
    echo
    echo "The board.settings file must define these parameters:"
    echo " CFG_[APP;DRV]_EPCS ..... Define to != \"\" if EPCS flash is available"
    echo "                          for the application or driver processor."
    echo
    echo
    echo "******************************************"
    echo " Example board.settings with EPCS connected to driver:"
    echo "  CFG_APP_EPCS="
    echo "  CFG_DRV_EPCS=1"
    echo "******************************************"
    echo
    exit 1
fi

if [ ! -d "${BOARD_PATH}" ];
then
    echo "ERROR: No path to hardware project specified!"
    exit 1
fi

BOARD_SETTINGS_FILE=${BOARD_PATH}/board.settings

if [ ! -f "${BOARD_SETTINGS_FILE}" ];
then
    echo "ERROR: No board.settings file found!"
    exit 1
fi

# Source the board settings file
CFG_APP_EPCS=
CFG_DRV_EPCS=
source ${BOARD_SETTINGS_FILE}

if [ -n "${CFG_APP_EPCS}" ] && [ -z "${CFG_DRV_EPCS}" ]; then
    CFG_EPCS_CPU=${CFG_APP_CPU_NAME}
elif [ -z "${CFG_APP_EPCS}" ] && [ -n "${CFG_DRV_EPCS}" ]; then
    CFG_EPCS_CPU=${CFG_DRV_CPU_NAME}
elif [ -n "${CFG_APP_EPCS}" ] && [ -n "${CFG_DRV_EPCS}" ]; then
    echo "ERROR: This script can generate the bootloader files only for one"
    echo "       CPU instance. Make sure that only one EPCS device is included"
    echo "       in the board design. Also revise the board.settings file!"
    exit 1
else
    echo "ERROR: The selected board's design does not include an EPCS flash device."
    echo "       Make sure that an EPCS device is available and set CFG_[APP;DRV]_EPCS"
    echo "       in the board.settings file!"
    exit 1
fi

# Get the sopcinfo file
SOPCINFO_FILE=$(find ${BOARD_PATH} -name *.sopcinfo)

# Check if the argument is a file
if [ ! -f "${SOPCINFO_FILE}" ];
then
    echo "ERROR: No SOPCINFO file found in path ${BOARD_PATH}"
    echo "       -> Generate Qsys to get SOPCINFO file!"
    exit 1
fi

# Convert sopcinfo to swinfo
CMD="sopcinfo2swinfo --input=${SOPCINFO_FILE} --output=${SWINFO_FILE}"
$CMD || {
    echo "ERROR: sopcinfo2swinfo failed!"
    exit 1
}

# convert swinfo to settings file (shell format)
if [ -n "${CFG_EPCS_CPU}" ];
then
    ARG="--module ${CFG_EPCS_CPU}"
fi

CMD="swinfo2header --swinfo ${SWINFO_FILE} --format sh --single ${SETTINGS_FILE} ${ARG}"
$CMD || {
    echo "ERROR: swinfo2header failed!"
    exit 1
}

# Now source the generated settings file
EPCS_FLASH_CONTROLLER_BASE=
REMOTE_UPDATE_BASE=

source ${SETTINGS_FILE}

# Check if EPCS flash controller is present
if [ -z "${EPCS_FLASH_CONTROLLER_BASE}" ];
then
    echo "ERROR: No EPCS flash controller found in SOPCINFO!"
    echo "       Make sure that the EPCS flash controller is named: epcs_flash_controller"
    exit 1
fi

# Check if Remote Update Core is present
if [ -z "${REMOTE_UPDATE_BASE}" ];
then
    echo "ERROR: No Remote Update controller found in SOPCINFO!"
    echo "       Make sure that the Remote Update controller is named: remote_update"
    exit 1
fi

# Now, collect all needed info
CODE_BASE=${EPCS_FLASH_CONTROLLER_BASE}
EPCS_BASE=${EPCS_FLASH_CONTROLLER_BASE}
EPCS_REGS_OFFS=${EPCS_FLASH_CONTROLLER_REGISTER_OFFSET}

# Get base of EPCS registers (also convert back to hexadecimal format)
EPCS_REGS_BASE=$(printf "0x%X" $(( ${EPCS_BASE} + ${EPCS_REGS_OFFS} )))

# And invoke makefile
ARG="\
CODE_BASE=${CODE_BASE} \
EPCS_REGS_BASE=${EPCS_REGS_BASE} \
REMOTE_UPDATE_BASE=${REMOTE_UPDATE_BASE} \
"

echo
echo "Call make with setting options:"
for i in $ARG
do
    echo " $i"
done

CMD="make clean all"
echo
echo $CMD
${CMD} ${ARG} || {
    echo "ERROR: make failed!"
    exit 1
}

# Remove swinfo and settings file
rm -rf ${SWINFO_FILE} ${SETTINGS_FILE}

# Search for the EPCS flash boot rom hex in the Quartus directory
QUARTUS_HEX_FILE=$(find ${BOARD_PATH} -name *epcs_flash_controller_boot_rom.hex)

if [ -f "${QUARTUS_HEX_FILE}" ];
then
    echo "INFO: Replace hex file in Quartus directory with newly generated one."
    CMD="cp -f ${HEX_FILE} ${QUARTUS_HEX_FILE}"
    echo
    echo $CMD
    $CMD || {
        echo "ERROR: Failed to copy file!"
        exit $?
    }
else
    echo "ERROR: Couldn't find the EPCS Flash boot rom hex file to replace it!"
    exit 1
fi

echo
echo "INFO: Script done successfully!"
exit 0
