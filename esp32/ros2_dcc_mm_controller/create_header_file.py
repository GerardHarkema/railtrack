import json
from datetime import datetime


track_config_file = '../../config/track_config_DCC.json'
with open(track_config_file, 'r', encoding='utf-8') as f:
    track_config = json.load(f)


header = "// !!! This is an automated generated header file, do not modify by your self !!!\n"

# datetime object containing current date and time
now = datetime.now()
line = "// Timestamp: " + now.strftime("%d/%m/%Y %H:%M:%S") + "\n"
header = header + line
line = "#ifndef _TRACK_CONFIG_\n"
header = header + line
line = "#define _TRACK_CONFIG_\n"
header = header + line + "\n"


line = "// Track config generated from: " + track_config_file + "\n"
header = header + line
try:
    locomotives = track_config["Locomotives"]
    #print(locomotives)

    line = "LOCOMOTIVE active_locomotives[] = {"
    for i in range(len(locomotives) - 1):
        line = line + "{" + str(locomotives[i]["address"]) + ', '
        line = line + "railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_" + str(locomotives[i]["protocol"]) + ', '
        if str(locomotives[i]["protocol"]) == "DCC":
            speed_steps = locomotives[i]["speed_steps"]
            if speed_steps == 128:
                line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_128" + '}, '    
            elif speed_steps == 28:
                line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_28" + '}, '    
            elif speed_steps == 14:
                line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_14" + '}, '  
            else:
                line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN" + '}, '  
        else:
            line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN" + '}, '  
        #line = line + "0},"

    line = line + "{" + str(locomotives[len(locomotives) - 1]["address"]) + ', '
    line = line + "railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_" + str(locomotives[len(locomotives) - 1]["protocol"]) + ', '
    if str(locomotives[len(locomotives) - 1]["protocol"]) == "DCC":
        speed_steps = locomotives[len(locomotives) - 1]["speed_steps"]
        if speed_steps == 128:
            line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_128" + '}, '    
        elif speed_steps == 28:
            line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_28" + '}, '    
        elif speed_steps == 14:
            line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_14" + '}, '  
        else:
            line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN" + '}, '  
    else:
        line = line + "railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN" + '}, '    
    line = line + "};\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   " + str(len(locomotives)) + "\n"
    header = header + line + "\n"
except KeyError:
    line = "//!!! Dummy pointer to active_locomotives !!!\n"
    header = header + line
    line = "LOCOMOTIVE *active_locomotives;\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   0\n"
    header = header + line + "\n"

#try:
turnouts = track_config["Turnouts"]
number_of_mm_turnouts = 0;
for turnout in turnouts:
    if (turnout["protocol"] == "MM1") or (turnout["protocol"] == "MM2"):
        number_of_mm_turnouts = number_of_mm_turnouts + 1;

if number_of_mm_turnouts:
    line = "unsigned short int active_turnouts_mm[] = {"
    for turnout in turnouts:
        if (turnout["protocol"] == "MM1") or (turnout["protocol"] == "MM2"):
            line = line + str(turnout["number"]) + ", "
    line  = line + "};\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_MM   " + str(number_of_mm_turnouts) + "\n"
    header = header + line  + "\n"
else:
    line = "//!!! Dummy pointer to active_turnouts_mm !!!\n"
    header = header + line
    line = "unsigned short int *active_turnouts_mm;\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_MM  0\n"
    header = header + line  + "\n"


number_of_ros_turnouts = 0;
for turnout in turnouts:
    if turnout["protocol"] == "ROS":
        number_of_ros_turnouts = number_of_ros_turnouts + 1;

if number_of_ros_turnouts:
    line = "unsigned short int active_turnouts_ros[] = {"
    for turnout in turnouts:
        if turnout["protocol"] == "ROS":
            line = line + str(turnout["number"]) + ", "
    line  = line + "};\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_ROS   " + str(number_of_ros_turnouts) + "\n"
    header = header + line  + "\n"
else:
    line = "//!!! Dummy pointer to active_turnouts_mm !!!\n"
    header = header + line
    line = "unsigned short int *active_turnouts_mm;\n"
    header = header + line
    line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_MM  0\n"
    header = header + line  + "\n"

line = "#endif //_TRACK_CONFIG_\n"
header = header + line

with open('include/track_config_old.h', 'w', encoding='utf-8') as f:
    f.write(header)

