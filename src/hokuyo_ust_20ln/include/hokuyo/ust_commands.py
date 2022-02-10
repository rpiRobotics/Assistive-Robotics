# This file contains the list of the commands used by the Hokuyo UST
# Each constant will be used as a message receiver or sender parameter (Send speed, read speed, etc..)
# NOTICE: These constants are for serial use
from collections import namedtuple

Command = namedtuple('Command',['command', 'answer_expected', 'answer'])

START_RANGING = Command('#GT15466', False, '')
STOP_RANGING = Command('#ST5297', True, '#ST00A845')
ID = Command('#IN0D54', True, '')
ID2 = Command('#CLC2DD', True, '')

PLOP = Command('#GR0EEE1', True, '')

MEASURE_LENGHT_UST_20LN = 8679 # For ust-20LN  8679 bytes = 1081steps *8bytes + 31 bytes 
MEASURE_STEPS_UST_20LN = 1081

MEASURE_LENGHT_UST_05LN = 4359 # For ust-05LN  4359 bytes = 541steps * 8bytes + 31 bytes 
MEASURE_STEPS_UST_05LN = 541
