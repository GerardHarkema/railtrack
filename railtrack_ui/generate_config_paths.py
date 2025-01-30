#!/usr/bin/env python3
import sys

# Read in the template file
with open(sys.argv[1] + '/launch/railtrack_ui.launch.py.template', 'r') as file:
  filedata = file.read()


railtrack_ui_path = "\"" +  sys.argv[1] + "\""
filedata = filedata.replace("<railtrack_ui_path>", railtrack_ui_path)


# Write the file launch file
with open(sys.argv[1] + '/launch/railtrack_ui.launch.py', 'w') as file:
  file.write(filedata)


# Read in the template file
with open(sys.argv[1] + '/launch/railtrack_emulator.launch.py.template', 'r') as file:
  filedata = file.read()

# Replace the target locomotive_images_path string
railtrack_ui_path = "\"" +  sys.argv[1] + "\""
filedata = filedata.replace("<railtrack_ui_path>", railtrack_ui_path)

# Write the file launch file
with open(sys.argv[1] + '/launch/railtrack_emulator.launch.py', 'w') as file:
  file.write(filedata)
