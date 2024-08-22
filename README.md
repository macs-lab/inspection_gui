## EOAT Hardware Setup

![](docs/hardware.drawio.png)

Materials
- [PoE Injector](https://www.amazon.com/TP-LINK-TL-PoE150S-Injector-Adapter-Compliant/dp/B001PS9E5I/ref=sr_1_7?crid=3SG1FRCW1W49T&dib=eyJ2IjoiMSJ9.FT_Yk-7vUIP39fYOXnsYFm66nv-sipdtsHa2XD-V1RypL-TOL1ARi2Fs67QyQiOggXMjiAuaWL8P1SCbqaTkGEzbEZ2Yd1luaCgUmVsXrxuQ6bJnNwvMiRms-soLQEd-k8maKuPFlqlZMY-8Y0Krtufef3ap_KVgY2-32ah3oBs6gLDWFJ4HY6GaVtEb5VejWdit0L_VBn6g5UkRxnMXhpHYsxkmM9qex9_ekTwnFmc.Jym5OS86b_dgeBSJ3p5Rzn8AT5QN_XHyVG0VjrUXaxU&dib_tag=se&keywords=PoE+injector&qid=1723161077&sprefix=poe+injecto%2Caps%2C164&sr=8-7)
- [RPi PoE Hat](https://www.amazon.com/LoveRPi-Power-Over-Ethernet-Raspberry-Professional-Isolated/dp/B07XB5PR9J/ref=sr_1_1_pp?crid=1PXZZ81ABY7DJ&dib=eyJ2IjoiMSJ9.iAkCGeJwEvj8Ie_ipuLCBwVCxvi9E_qxH4MvANR6NyZyjc8NE-qnvlsa_dd_nLqadgYxbymn_7vGGP1YnjhS1QzM2LwQywK357qwAMrIdxA9Yj2RMRtxzjnZXiIBbuPH67ZhuXJU85YQwMEfNM47Beo-kaB_UqdsTkHfJaKJ_7xATjOWd_3w5NdWJ9VKneNN7UpkwoFO9pKH2JOFXqQI4ask_N9ZNOC4JuALHTdfHOI.DHyFm5AqkubkuGlRrhPdKZBhWb16btyeUXFbNNyR4kk&dib_tag=se&keywords=raspberry+pi+4b+poe+hat&qid=1723161129&sprefix=raspberry+pi+4b+poe+hat%2Caps%2C200&sr=8-1)
- Raspberry Pi 4B Case compatible with PoE Hat
- Cat 8 Ethernet Cable (Hub to PoE Injector)
- Cat 8 Ethernet Cable (long enough to go from PoE Injector to robot's end effector)
- Cat 8 Ethernet Cable (Hub to PC)
- Short USB 2.0 to USB Mini-B Cable (RPi to Camera)

## Install Ubuntu Server 22.04

Install Raspberry Pi Imager

[Windows or macOS](https://www.raspberrypi.com/software/)

Debian
```bash
sudo apt install rpi-imager
```

1) Open Raspberry Pi Imager

2) Select operating system:
Choose OS -> Other General Purpose OS -> Ubuntu -> Ubuntu Server 22.04 LTS (64-bit).

3) Select storage device for OS.

4) Click on settings icon in bottom right and use [this configuration](docs/imager_settings.png).

5) Flash device

Insert micro SD card into Raspberry Pi and connect to power. To perform the rest of the setup, we need to get the Pi's IP so we can connect. To find this, connect the Pi to a keyboard and monitor. Login and run the following commands:

```bash
sudo apt install net-tools
ifconfig
```

Make note of the ip of the form 192.168.1.x.

## EOAT Setup

### SSH into Pi

```bash
ssh pi@192.168.1.x
```

Type "yes" if asked to verify signature of device. Input password to login. In the future you should be able to SSH using the hostname of the Pi with the command `ssh pi@inspectioneoat.local`

### Install docker
[Install Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
docker run hello-world
```
If install was successful, you should see [this output](docs/docker_install_output.png) after running the last command.


### Clone repository
```
git clone https://github.com/cacton77/inspection_eoat.git
```

Build Image
```bash
cd inspection_eoat
docker compose build
docker compose up lights camera
```
