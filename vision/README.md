# Vision Model

## Vision - setup

In order to run the vision system, you need to have a few dependencies installed, this can usually be done by first you have brew installed on mac (note that you'll have this installed already if you have run the controller.py):

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
(echo; echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"') >> /home/arch/.bashrc
sudo pacman -S base-devel
eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"
```

Now that brew is installed, you may change to the folder `src/vision/rgbdSeg` and run the `build.sh` with an option:

```bash
cd src/vision/rgbdSeg
bash build.sh --include-deps
```

This should automatically install all the missing dependencies and then compile the executable `planeSegment`.

## Profiling 

Using activity monitor and intruments in mac, I was able to determine the CPU and memory usage of each of the components of both the vision and language models. Here is a summary:

__Vision__

| Component         | CPU Usage (max, %) | CPU Usage (threads) | Memory Usage (max, MB)    |
|--------------|-----|-----|---------------|
| controller.py        | 105  | 32  | 500      |
| rasa-model          | 25  |  22 | 900      |
| rasa-actions      | 1  | 2 | 80      |