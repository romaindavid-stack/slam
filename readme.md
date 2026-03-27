# SLAM

Complete folder to run the lidar (livox), the gps and slam in one click.

## Setup

### ethernet port config for livox lidar
We need to set a custom setting on the laptop ethernet port so it can communicate with the LiDAR.  
Settings -> Network -> PCI Ethernet -> Settings (cog wheel) -> IPV4 -> Manual (IPV4 method) -> Addresses:  
Change the address to 192.168.1.50 and netmask to 255.255.255.0  

### install

```bash
./setup.sh
source install/setup.bash
```

To test succesful, no fatal errors should happen, and the following command should return a path:
```bash
ros2 pkg prefix my_configs
```

### launch
You can launch either with or without recording in a ros2bag. If you record, always launch from the slam directory.  
Argumets:  
- record (default `false`)
- playback (default `false`)
- bagfile (path to the bag if playback set to `true`)
```bash
ros2 launch my_configs fast_lio_deploy.launch.py
ros2 launch my_configs fast_lio_deploy.launch.py record:=true
ros2 launch my_configs fast_lio_deploy.launch.py playback:=true bag_file:="/home/romain/ros2_data/recordings/keith_livox1"
```


## additional info

### folder tree
```
slam/  
├── build
│   ├── ...
├── install
│   ├── ...
├── log
│   ├── ...
└── src
    ├── FAST_LIO
    ├── livox_ros_driver2
    ├── Livox-SDK2
    ├── my_configs
    ├── ntrip_client
    └── ublox
├── setup.sh                     # The "Magic" script  
└── README.md  
```
### detailled my config

```
my_configs/
├── config/
│   ├── mid360.yaml          # For Fast-LIO
│   └── MID360_config.yaml   # For the Livox Driver
├── launch/
│   └── fast_lio_deploy.launch.py
├── resource/
│   └── my_configs           # An empty file with the package name inside
├── package.xml
├── setup.py
└── setup.cfg
```
