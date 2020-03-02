
# Some installs

```bash
apt-get install libusb-1.0-0-dev cmake
wget https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz
# install libphidget according to that package. phidgets21 was used previously in case there are errors.
# install the 60-phidgets.udev rule to /etc
```

Then use CMake to set up the package and compile `test_darwin.cpp` as example code.

# Code

Primary hardware interface is through `interface.h`, while we have a MuJoCo interface through `sim_interface.h`.

Files that interface with sensors are prepended with s_, as in `s_imu.h`.
