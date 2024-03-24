# TAS5805M DAC running Raspbian on Raspberry Pi (Zero, Zero W, Zero 2W)

This repository contains the device tree and source for `tas5805m` kernel module for Raspberry Pi Zero running under Raspbian. I prepared this repo to do it quickly

So first get it from GitHub (all commands going forward are running on target host, ie Raspberry Pi) 

```
git clone https://github.com/sonocotta/tas5805m-for-raspbian-paspberry-pi-zero && cd tas5805m-for-raspbian-paspberry-pi-zero
```

We're about to build kernel modules, so we need to install few dependencies first 

```
sudo apt install raspberrypi-kernel-headers build-essentials -y
```

## Device tree 

We need to add user overlays to enable I2S  (disabled by default) and enable a sound card on that port

```
sudo compile-overlay.sh
```

this will compile the overlay file, put the compiled file under `/boot/overlays` 

Next add to `/boot/config.txt`

```
# Enable DAC
dtoverlay=tas5805m,i2creg=0x2d
```

You may also comment out built-in audio and HDMI if you're running headless

```
#dtparam=audio=on
#dtoverlay=vc4-kms-v3d
```

You need to reboot for changes to take effect, but this will not work just yet. The problem is we are referencing the `tas5805m` kernel module there, and this one is not included in Raspbian. Therefore we will build it on the same board using current kernel sources that we just installed 

## Kernel module

Now you are ready to build. The first command produces `ras5805m.ko` file among others. Second will copy it to the appropriate kernel modules folder.

```
make all
sudo make install
```

Now we are ready to reboot and check if we have a sound card listed

```
**** List of PLAYBACK Hardware Devices ****
card 0: LouderRaspberry [Louder-Raspberry], device 0: bcm2835-i2s-tas5805m-amplifier tas5805m-amplifier-0 [bcm2835-i2s-tas5805m-amplifier tas5805m-amplifier-0]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

Check if new module is correctly loaded

```
$ lsmod | grep tas5805m
tas5805m                6269  1
regmap_i2c              5027  1 tas5805m
snd_soc_core          240140  4 snd_soc_simple_card_utils,snd_soc_bcm2835_i2s,tas5805m,snd_soc_simple_card
```

Now quick test if audio is functional

```
speaker-test -t sine -f 500 -c 2
```

And finally, I can hear a beeping sound in both speakers. Hooray!
