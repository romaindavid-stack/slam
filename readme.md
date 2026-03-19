# SLAM

Complete folder to run slam in one click.

## Setup

### ethernet port config
We need to set a custom setting on the laptop ethernet port so it can communicate with the LiDAR.  
Settings -> Network -> PCI Ethernet -> Settings (cog wheel) -> IPV4 -> Manual (IPV4 method) -> Addresses:  
Change the address to 192.168.1.50 and netmask to 255.255.255.0  

### install

```bash
./setup.sh
```


## additional info

### folder tree

slam/
├── src/
│   ├── drivers/
│   │   └── Livox-SDK2/          # Submodule (Code)
│   ├── slam/
│   │   └── FAST_LIO/            # Submodule (Code)
│   └── my_configs/              # YOUR custom ROS 2 Package
│       ├── config/
│       │   ├── custom_mid360.yaml
│       │   └── fast_lio_params.yaml
│       └── launch/
│           └── fast_lio_deploy.launch.py
├── setup.sh                     # The "Magic" script
└── README.md