#!/usr/bin/env python3
import os

user = os.getlogin()

current_directory = os.getcwd()
dir, file = os.path.split(current_directory)
# Change dir two steps to root, that should be the place of the workspace 
dir, file = os.path.split(dir)
workspace_path, file = os.path.split(dir)
print(workspace_path)


# Read in the template file
with open('railtrack_control.service.template', 'r') as file:
  filedata = file.read()

filedata = filedata.replace("<user>", user)
filedata = filedata.replace("<workspace_path>", workspace_path)

# Write the file service file
with open('railtrack_control.service', 'w') as file:
  file.write(filedata)

# Read in the template file
with open('railtrack_gui.service.template', 'r') as file:
  filedata = file.read()

filedata = filedata.replace("<user>", user)
filedata = filedata.replace("<workspace_path>", workspace_path)

# Write the file service file
with open('railtrack_gui.service', 'w') as file:
  file.write(filedata)