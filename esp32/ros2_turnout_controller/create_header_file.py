import json
from datetime import datetime

try:
    import easygui                    
except ImportError:
    Import("env")
    # Install custom packages from the PyPi registry
    env.Execute("$PYTHONEXE -m pip install easygui")
    import easygui


def create_turnout_config_struct_line(turnout):
    if turnout["type"] == "magnet":
        line = "\t\t{MAGNET, " + str(turnout["number"]) + ', '
        line = line + "{.magnet = {"
        line = line + str(turnout["red_pin"]) + ', '
        line = line + str(turnout["green_pin"])+ "}}},\n"
    elif turnout["type"] == "servo":
        line = "\t\t{SERVO, " + str(turnout["number"]) + ', '
        line = line + "{.servo = {"
        line = line + str(turnout["pin"]) + ', '
        line = line + str(turnout["green_value"]) + ', '
        line = line + str(turnout["red_value"])+ "}}},\n"
    elif turnout["type"] == "analog_out":
        line = "\t\t{ANALOG_OUT, " + str(turnout["number"]) + ', '
        line = line + "{.analog_out = {"
        line = line + str(turnout["pin"]) + ', '
        line = line + str(turnout["green_value"]) + ', '
        line = line + str(turnout["red_value"])+ "}}},\n"
    elif turnout["type"] == "digital_out":
        line = "\t\t{DIGITAL_OUT, " + str(turnout["number"]) + ', '
        line = line + "{.digital_out = {"
        line = line + str(turnout["pin"]) + ', '
        if turnout["negative_logic"]:
            line = line + "true}}},\n"
        else:
            line = line + "false}}},\n"

    else:
        print("Invalid turnout type defined")
    return line


def main():

    header = "// !!! This is an automated generated header file, do not modify by your self !!!\n"

    # datetime object containing current date and time
    now = datetime.now()
    line = "// Timestamp: " + now.strftime("%d/%m/%Y %H:%M:%S") + "\n"
    header = header + line
    line = "#ifndef _TURNOUT_CONFIG_\n"
    header = header + line
    line = "#define _TURNOUT_CONFIG_\n"
    header = header + line + "\n"


    load_title = "Load turnout config from..."
    turnout_config_file = easygui.fileopenbox(title=load_title, default="../../config/*.json")
    if turnout_config_file is not None:
        with open(turnout_config_file, 'r', encoding='utf-8') as f:
            turnout_config = json.load(f)
    else:
        easygui.msgbox('Turnout controller will be compiled with previous turnout_config.h', 'Warning')
        return

    #turnout_config_file = '../../config/turnout_config_a.json'
    line = "// Turnout config generated from: " + turnout_config_file + "\n"
    header = header + line

    node_name = turnout_config["Node_name"]
    line = "#define  NODE_NAME  \"" + node_name + "\"\n"
    header = header + line + "\n"
    try:
        status_led_pin = turnout_config["Status_led_pin"]
        line = "#define  STATUS_LED  " + str(status_led_pin) + "\n"
    except KeyError:
        line = "#define  STATUS_LED  LED_BUILTIN\n"
    header = header + line + "\n"

    turnouts = turnout_config["Turnouts"]
    line = "TURNOUT_CONFIG turnout_config[] = {"
    header = header + line + "\n"
    for turnout in turnouts:
        header = header + create_turnout_config_struct_line(turnout)
    line = "\t\t};\n"
    header = header + line
    line = "#define  NUMBER_OF_TURNOUTS  " + str(len(turnouts)) + "\n"
    header = header + line + "\n"

    line = "#endif //_TURNOUT_CONFIG_\n"
    header = header + line

    with open('include/turnout_config.h', 'w', encoding='utf-8') as f:
        f.write(header)

main()