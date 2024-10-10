import json
from datetime import datetime

try:
    import easygui                    
except ImportError:
    Import("env")
    # Install custom packages from the PyPi registry
    env.Execute("$PYTHONEXE -m pip install easygui")
    import easygui


def create_scenery_config_struct_line(scenery):
    if scenery["type"] == "magnet":
        line = "\t\t{MAGNET, " + str(scenery["number"]) + ', '
        line = line + "{.magnet = {"
        line = line + str(scenery["red_pin"]) + ', '
        line = line + str(scenery["green_pin"])+ "}}},\n"
    elif scenery["type"] == "servo":
        line = "\t\t{SERVO, " + str(scenery["number"]) + ', '
        line = line + "{.servo = {"
        line = line + str(scenery["pin"]) + ', '
        line = line + str(scenery["green_value"]) + ', '
        line = line + str(scenery["red_value"])+ "}}},\n"
    elif scenery["type"] == "analog_out":
        line = "\t\t{ANALOG_OUT, " + str(scenery["number"]) + ', '
        line = line + "{.analog_out = {"
        line = line + str(scenery["pin"]) + ', '
        line = line + str(scenery["green_value"]) + ', '
        line = line + str(scenery["red_value"])+ "}}},\n"
    elif scenery["type"] == "digital_out":
        line = "\t\t{DIGITAL_OUT, " + str(scenery["number"]) + ', '
        line = line + "{.digital_out = {"
        line = line + str(scenery["pin"]) + ', '
        if scenery["negative_logic"]:
            line = line + "true}}},\n"
        else:
            line = line + "false}}},\n"

    else:
        print("Invalid scenery type defined")
    return line


def main():

    header = "// !!! This is an automated generated header file, do not modify by your self !!!\n"

    # datetime object containing current date and time
    now = datetime.now()
    line = "// Timestamp: " + now.strftime("%d/%m/%Y %H:%M:%S") + "\n"
    header = header + line
    line = "#ifndef _SCENERY_CONFIG_\n"
    header = header + line
    line = "#define _SCENERY_CONFIG_\n"
    header = header + line + "\n"


    load_title = "Load scenery config from..."
    scenery_config_file = easygui.fileopenbox(title=load_title, default="../../config/*.json")
    if scenery_config_file is not None:
        with open(scenery_config_file, 'r', encoding='utf-8') as f:
            scenery_config = json.load(f)
    else:
        easygui.msgbox('Scenery controller will be compiled with previous scenery_config.h', 'Warning')
        return

    #scenery_config_file = '../../config/scenery_config_a.json'
    line = "// Scenery config generated from: " + scenery_config_file + "\n"
    header = header + line

    node_name = scenery_config["Node_name"]
    line = "#define  NODE_NAME  \"" + node_name + "\"\n"
    header = header + line + "\n"
    if 0:
        try:
            status_led_pin = scenery_config["Status_led_pin"]
            line = "#define  STATUS_LED  " + str(status_led_pin) + "\n"
        except KeyError:
            line = "#define  STATUS_LED  LED_BUILTIN\n"
        header = header + line + "\n"

        scenerys = scenery_config["Turnouts"]
        line = "SCENERY_CONFIG scenery_config[] = {"
        header = header + line + "\n"
        for scenery in scenerys:
            header = header + create_scenery_config_struct_line(scenery)
        line = "\t\t};\n"
        header = header + line
        line = "#define  NUMBER_OF_TURNOUTS  " + str(len(scenerys)) + "\n"
        header = header + line + "\n"

    line = "#endif //_SCENERY_CONFIG_\n"
    header = header + line

    with open('include/scenery_config.h', 'w', encoding='utf-8') as f:
        f.write(header)

main()