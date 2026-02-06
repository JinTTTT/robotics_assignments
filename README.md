## Environment setup
We setup enviroment for macos using docker.
### Docker setup
1. start colima
``` colima start ```
2. pull ubuntu 20.04 image, it automatically use arm64 based image, since we use apple silicon chip.
``` docker pull ubuntu:20.04 ```
3. start container, attach current directory to container, setup X11 forwarding and display environment variable. name as robotics_assignment5, interactive mode, terminal mode.
```
docker run -it \
    -v "/Users/jtao/Documents/Study/Computer Engineering/WS2026/Robotics/assignments/A5:/workspace" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --name robotics_assignment5 \
    ubuntu:20.04 \
    /bin/bash
```
4. check
check which ubuntu
``` cat /etc/os-release ```
check cpu architecture
``` uname -m ```

5. restart container
``` 
docker restart robotics_assignment5 
docker exec -it robotics_assignment5 /bin/bash
```

## Install dependencies
1. since we use minimal ubuntu image, it doesn't have somes tools we expect to have, we need to install them manually.
```
apt-get update
apt-get install -y software-properties-common
```
2. install robotics library
```
add-apt-repository ppa:roblib/ppa
apt-get update
apt-get

