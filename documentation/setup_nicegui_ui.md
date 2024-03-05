# Setup nicegui User Interface

Under Construction

The user interface can be started in two ways:
* Manualy:
```bash
ros2 launch railtrack_ui railtrack_ui.launch.py
```
* Using script: run __start_railtrack_gui.sh__ script in scripts directory of this repository.
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./start_railtrack_gui.sh
```
### Configuring the user interface
You can "tweak" the user interface according you wishes. Th only thing you have to do is editing the track_config.json file in the config directory of this repository.
The most relevant json-objects are described below.

### Common Controls & Indicators
At the bottom of the user interface there ar two items:
* Stop/Enable knob: Controls the main power of the railtrack
* Active LED: Indicates the connection activity between the user interface and the Marklin Railbox Bridge. In actif state it flashes.(if no Marklin Ralibox Bridge is used, this indicator has no function)

![Image](images/Screenshot%20Common%20Controls%20and%20Indicators.png)

### Locomotives tab
![Image](images/Screenshot%20Locomotives.png)

### Turnouts tab
![Image](images/Screenshot%20Turnouts.png)

### Track Layout tab
![Image](images/Screenshot%20Track%20Layout.png)


Alternative you can start the user interface by installing a Linux-service using the __install_railtrack_gui_service.sh__ script in the linux_service directory of this repository (note: the user interface might not be running, abort with ctrl-C).
```bash
cd ~/railtrack_ws/src/railtrack/linux_service
./install_railtrack_gui_service.sh
```

[Back](../README.md)