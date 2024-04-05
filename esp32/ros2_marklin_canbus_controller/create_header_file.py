import json
from datetime import datetime


track_config_file = '../../config/track_config.json'
with open(track_config_file, 'r', encoding='utf-8') as f:
    track_config = json.load(f)

agent_config_file = '../../config/micro_ros_agent_config.json'
my_agent_config_file = '../../../micro_ros_agent_config.json'

try:
    with open(my_agent_config_file, 'r', encoding='utf-8') as f:
        agent_config = json.load(f)
except OSError:
    with open(agent_config_file, 'r', encoding='utf-8') as f:
        agent_config = json.load(f)

header = "// !!! This is an automated generated header file, do not modify by your self !!!\n"

# datetime object containing current date and time
now = datetime.now()
line = "// Timestamp: " + now.strftime("%d/%m/%Y %H:%M:%S") + "\n"
header = header + line
line = "#ifndef _TRACK_CONFIG_\n"
header = header + line
line = "#define _TRACK_CONFIG_\n"
header = header + line + "\n"

line = "// Agent config generated from: " + agent_config_file + "\n"
header = header + line
line = "#define SSID   \"" + agent_config['wifi']['ssid'] + "\"\n"
header = header + line
line = "#define PASSWORD   \"" + agent_config['wifi']['password'] + "\"\n"
header = header + line

ip_address = agent_config['agent']['ip']
ip_segements = ip_address.split(":")
line = "uint8_t ip_address[4] = {" + ip_segements[0] + ", "\
                                   + ip_segements[1] + ", "\
                                   + ip_segements[2] + ", "\
                                   + ip_segements[3] + "};\n"
header = header + line

line = "#define PORT   " + str(agent_config['agent']['port']) + "\n\n"
header = header + line


line = "// Track config generated from: " + track_config_file + "\n"
header = header + line
try:
    locomotives = track_config["Locomotives"]
    #print(locomotives)

    line = "LOCOMOTIVE active_locomotives[] = {"
    for i in range(len(locomotives) - 1):
        line = line + "{" + str(locomotives[i]["address"]) + ', '
        line = line + str(locomotives[i]["protocol"]) + '}, '


    line = line + "{" + str(locomotives[len(locomotives) - 1]["address"]) + ', '
    line = line + str(locomotives[len(locomotives) - 1]["protocol"]) + '}, '
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

with open('include/track_config.h', 'w', encoding='utf-8') as f:
    f.write(header)

