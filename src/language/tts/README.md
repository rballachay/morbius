## Compiling the speak_audio.c file:

Only need to link to /opt/local if the package is installed with mac ports. Brew or manual build and install should install it to /usr/local.

bash```
 gcc speak_audio.c  -lespeak-ng -o test-audio -I /opt/local/include -L /opt/local/lib
```