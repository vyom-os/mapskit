# mapskit

`mapskit` package responsible for both C++ (`.cpp`) and Python (`.py`) nodes that interact with the ompl and octomap protocol.


# download 

```
mkdir -p ~/workspace/
cd ~/workspace/
git clone https://github.com/vyom-os/mapskit.git --recusive
```

# install dependencies
```
cd mapskit 
chmod +x setup.sh
sudo ./setup.sh
```

# build and install the package 

```
cd mapskit 
mkdir -p build 
cmake ..
make 
sudo make install
```
this installs the package for a systemwide location at `/usr/local/mapskit*`
for a more custom location change prefix path ny passing args to
```
cmake -DCMAKE_PREFIX_PATH=/mycustom-install-path ..
```
and then proceed with make as usual. 

# running 

Kindly go though the docs to understand what changes to make and how they affect the system performance, orbbec camera is used here as an example.
* [understanding voxels](orbbec_voxels.md) 
* [understanding tracking](orbbec_tracking.md)

this essentially replaces octomap-server with an enhanced mapskit server built on top of it offering additional features and APIs.

## setting up your custom config
you can find the default configs at `/usr/local/share/mapskit/config/default_conf.yaml`
to set your custom configs you can use the cp the default and make changes 
```
mkdir -p ~/.mapskit/
cp /usr/local/share/mapskit/config/default_conf.yaml ~/.mapskit/config.yaml
```
change input / output, topic names as needed, tweak params etc.  

you can run a benchtest to verify the working by using 
```
./usr/local/lib/mapskit/mapskit_core_cxx /home/myuser/.mapskit/config.yaml
```
or just use default configs by running 
```
./usr/local/lib/mapskit/mapskit_core_cxx
```

## using a .service to use precompiled .cxx binary
create a .service file at /etc/system
```
sudo nano /etc/systemd/system/mapskit-server.service
```
and append the following content 
```
[Unit]
Description= mapskit-server Service
After=network.target

[Service]
ExecStart=/bin/bash -c "/usr/local/lib/mapskit/mapskit_core_cxx /home/myuser/.mapskit/config.yaml"
Restart=on-failure
User=myuser
Group=mygroup
WorkingDirectory=/opt/my_application

[Install]
WantedBy=multi-user.target
```

start and enable this by 
```
sudo systemctl daemon-reload
sudo systemctl enable --now mapskit-server.service 
```

## API integration - Add site-packages to PYTHONPATH 

try running a python3 shell and importing mapskit
```
python3
import mapskit
```

sometimes if the site install path is not there in pythonpath `import mapskit` *can fail*.

You can add it to `.bashrc` for a permanent fix 
```
if [[ ":$PYTHONPATH:" != *":/usr/local/lib/python3.10/site-packages:"* ]]; then
    export PYTHONPATH="/usr/local/lib/python3.10/site-packages${PYTHONPATH:+":$PYTHONPATH"}"
fi
```
and then retry. 


> find binaries in this location  `exec /usr/local/lib/mapskit/`
> python modules get installed in `/usr/local/lib/python3.10/site-packages`


# WIP / Roadmap : 

Fix and integrate remappings from topics i/o. 
Testcases for python3 bindings ...
