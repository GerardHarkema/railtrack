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
locomotives = track_config["Locomotives"]
#print(locomotives)

line = "LOCOMOTIVE active_locomotives[] = {"
for i in range(len(locomotives) - 1):
    line = line + "{" + str(locomotives[i]["address"]) + ', '
    line = line + str(locomotives[i]["protocol"]) + ', '
    line = line + "0},"

line = line + "{" + str(locomotives[len(locomotives) - 1]["address"]) + ', '
line = line + str(locomotives[len(locomotives) - 1]["protocol"]) + ', '
line = line + " 0}};\n"
header = header + line
line = "#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   " + str(len(locomotives)) + "\n"
header = header + line + "\n"

turnouts = track_config["Turnouts"]["railbox_controlled"]
line = "unsigned short int active_turnouts_railbox[] = {"
for i in range(len(turnouts)-1):
    line = line + str(turnouts[i]["number"]) + ", "
line  = line + str(turnouts[len(turnouts)-1]["number"]) + "};\n"
header = header + line
line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_RAILBOX   " + str(len(turnouts)) + "\n"
header = header + line  + "\n"

turnouts = track_config["Turnouts"]["ros_controlled"]
line = "unsigned short int active_turnouts_ros[] = {"
for i in range(len(turnouts)-1):
    line = line + str(turnouts[i]["number"]) + ", "
line  = line + str(turnouts[len(turnouts)-1]["number"]) + "};\n"
header = header + line
line = "#define  NUMBER_OF_ACTIVE_TURNOUTS_ROS   " + str(len(turnouts)) + "\n"
header = header + line + "\n"

line = "#endif //_TRACK_CONFIG_\n"
header = header + line

with open('include/track_config.h', 'w', encoding='utf-8') as f:
    f.write(header)

