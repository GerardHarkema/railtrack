#!/usr/bin/env python3
import sys

# Read in the template file
with open(sys.argv[1] + '/launch/railtrack_ui.launch.py.template', 'r') as file:
  filedata = file.read()

# Replace the target config_file string
config_file =  "\"" + sys.argv[1] + "/../config/track_config.json\""
filedata = filedata.replace("<config_file>", config_file)

# Replace the target locomotive_images_path string
locomotive_images_path = "\"" +  sys.argv[1] + "/../config/locomotive_images\""
filedata = filedata.replace("<locomotive_images_path>", locomotive_images_path)

# Replace the target locomotive_images_path string
railtracklayout_images_path = "\"" +  sys.argv[1] + "/../config/railtracklayout_images\""
filedata = filedata.replace("<railtracklayout_images_path>", railtracklayout_images_path)

# Write the file launch file
with open(sys.argv[1] + '/launch/railtrack_ui.launch.py', 'w') as file:
  file.write(filedata)


# Read in the template file
with open(sys.argv[1] + '/launch/railtrack_emulator.launch.py.template', 'r') as file:
  filedata = file.read()

# Replace the target config_file string
config_file =  "\"" + sys.argv[1] + "/../config/track_config.json\""
filedata = filedata.replace("<config_file>", config_file)

# Replace the target locomotive_images_path string
locomotive_images_path = "\"" +  sys.argv[1] + "/../config/locomotive_images\""
filedata = filedata.replace("<locomotive_images_path>", locomotive_images_path)

# Replace the target locomotive_images_path string
railtracklayout_images_path = "\"" +  sys.argv[1] + "/../config/railtracklayout_images\""
filedata = filedata.replace("<railtracklayout_images_path>", railtracklayout_images_path)

# Write the file launch file
with open(sys.argv[1] + '/launch/railtrack_emulator.launch.py', 'w') as file:
  file.write(filedata)
