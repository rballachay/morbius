# Setup

See below the instructions for setting up the raspberry pi.

## Raspberry pi imager

In order to set up a raspberry pi from scratch, you must first install the OS to the SD card (we are using a [SanDisk 64GB Extreme PRO](https://www.amazon.ca/dp/B09X7BYSFG?psc=1&ref=ppx_yo2ov_dt_b_product_details)). You can install the [raspberry pi imager](https://www.raspberrypi.com/software/) for your OS and then run the setup. We are using a Raspberry Pi 5 with 64-bit Raspberry Pi OS. Set up the wifi credentials for your [local network + allow SSH](https://www.raspberrypi.com/documentation/computers/getting-started.html#raspberry-pi-imager). You will find this under the OS customization window. Set the username to: `user` and the password as you like. Once this is installed to the SD card, place the card into the raspberry Pi, attach the pi to your laptop with USB and scan the network until you can see raspberry pi listed (`sudo nmap -sn 192.168.2.0/24` on mac, but the IP will depend on your router). If you cannot, you will need to attach an ethernet cable to the pi and the router, and run the same `nmap` command again. Now SSH into the raspberry pi and set up the wifi through rasp config:

```bash
ssh user@raspberrypi.local 
# the password is test12345 
sudo raspi-config
# now update the wifi to include the router
sudo reboot
# follow all the other instructions to set up the wifi
```

## Setting up environment

Git is installed by default, so we can use this command to access all our code and set up the functions we will need:

```bash
git clone https://github.com/rballachay/morbius
cd morbius
chmod +x rasp/setup.sh
sudo bash rasp/setup.sh
```

## Running planeSegment using x11

You may run the planeSegment function and display the output window(s) to your local computer over ssh. In order to do this, you must ensure a few things locally and then ssh into the computer.


```bash
# if on linux
sudo apt-get install xauth
# if on mac
brew install xauth

ssh -X rileyballachay@raspberrypi.local 
sudo apt-get install xorg
sudo apt-get install openbox
```

Nano into the file `/etc/ssh/sshd_config`: (`sudo nano /etc/ssh/sshd_config`) and ensure that the following lines are all enabled:

```
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost no
```

And ensure that the following rules are set on your client computer (the one you're on). Inside of `~/.ssh/config`

```
Host *
  ForwardAgent yes
  ForwardX11 yes
```

If you run the script now, inside of `src/vision/rgbdSeg`: `./planeSegment`, you should be able to see the output in GUI windows on your host laptop.

## Checking space left on file system

After installing everything, we can verify the amount of space left on the filesystem using the following: `df -Bm`. Look for the filesystem mounted to `/`. In this case, we have about 59 GB available, and 13 GB was used when installing the planeSegment application and language controller.py.

## Increasing the swap size

One problem that was frequently encountered during development was running out of memory and the system crashing. To reduce the frequency of this occurance, you can increase the swap size (the portion of disk that is used as memory when RAM is exceeded) by running the following inside of raspberry pi ssh:

```bash
sudo dphys-swapfile swapoff

sudo dphys-swapfile setup
# find the following line: CONF_SWAPSIZE=100 
# and change to desired value, like CONF_SWAPSIZE=2048

sudo dphys-swapfile setup
sudo dphys-swapfile swapon
sudo reboot
```

Then you can ssh back into the raspberry pi and try running your program.

## Dealing with voltage issues

It is possible that when you connect to a power source, you aren't supplying adequate voltage. This was the case when I started using ORB_SLAM3 with the Intel Realsense D415 with the camera plugged into my macbook. You can determine if the pi is not supplied adequate voltage with the following command: `vcgencmd get_throttled`. This should return `throttled=0x0` if all is going well and `throttled=0x5000` in the case something is wrong, which was my case. If it doesn't  recieve enough voltage, the system can/will crash.