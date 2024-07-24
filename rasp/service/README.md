# Using systemd in raspberry pi 

I tried to create a service that launches automatically with the raspberry pi when its plugged in, but this proved to be way more difficult that I planned. What works better is that whoever is SSH-ed into the robot starts the program in a headless fasion like:

```bash
conda activate python310
python3 daemon.py > /dev/null 2>&1 &
```

I don't think I was super far from figuring this out (with the scripts that are provided here), but you need to better understand how the sockets/services like pulseaudio.service and pulseaudio.socket are launched when the user logs in. 