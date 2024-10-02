import json
from datetime import datetime

try:
    import easygui                    
except ImportError:
    Import("env")
    # Install custom packages from the PyPi registry
    env.Execute("$PYTHONEXE -m pip install easygui")
    import easygui



def main():
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
    line = "#ifndef _NETWORK_CONFIG_\n"
    header = header + line
    line = "#define _NETWORK_CONFIG_\n"
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


    line = "#endif //_NETWORK_CONFIG_\n"
    header = header + line

    with open('include/network_config.h', 'w', encoding='utf-8') as f:
        f.write(header)

main()
