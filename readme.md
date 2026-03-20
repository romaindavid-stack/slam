# SLAM

Complete folder to run slam in one click.

## Setup

### ethernet port config
We need to set a custom setting on the laptop ethernet port so it can communicate with the LiDAR.  
Settings -> Network -> PCI Ethernet -> Settings (cog wheel) -> IPV4 -> Manual (IPV4 method) -> Addresses:  
Change the address to 192.168.1.50 and netmask to 255.255.255.0  

### install

```bash
source setup.sh
```

To test succesful, no fatal errors should happen, and the following command should return a path:
```bash
ros2 pkg prefix my_configs
```

### launch

```bash
ros2 launch my_configs fast_lio_deploy.launch.py
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
    └── my_configs
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
