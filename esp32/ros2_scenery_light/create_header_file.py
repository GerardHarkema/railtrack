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

    if scenery["type"] == "mono":
        line = "\t\t{LT_MONO, " + str(scenery["number"]) + ', '
        line = line + "{.mono = {"
        line = line + str(scenery["pin_number"])+ "}}},\n"
    elif scenery["type"] == "rgb":
        line = "\t\t{LT_RGB, " + str(scenery["number"]) + ', '
        line = line + "{.rgb = {"
        line = line + str(scenery["pin_number"]) + ', '
        line = line + str(scenery["number_of_leds"]) + ', '
        if scenery["color_order"] == "RGB":
            line = line + "LED_ORDER_RGB" + "}}},\n" 
        elif scenery["color_order"] == "GRB":
            line = line + "LED_ORDER_GRB" + "}}},\n"
        else:
            line = line + "LED_ORDER_RGB" + "}}},\n"
        # others need to be  implemented !!!!


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


    scenerys = scenery_config["Lights"]
    line = "SCENERY_LIGHT_CONFIG scenery_lights_config[] = {"
    header = header + line + "\n"
    for scenery in scenerys:
        header = header + create_scenery_config_struct_line(scenery)
    line = "\t\t};\n"
    header = header + line
    line = "#define  NUMBER_OF_SCENERY_LIGHTS " + str(len(scenerys)) + "\n"
    header = header + line + "\n"

    line = "#endif //_SCENERY_CONFIG_\n"
    header = header + line

    with open('include/scenery_config.h', 'w', encoding='utf-8') as f:
        f.write(header)

main()