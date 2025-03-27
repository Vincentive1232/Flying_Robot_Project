# Flying_Robot_Project

## Project Structure


## Clone and Build
First clone the repo and the submodules:
```
git clone --recurse-submodules https://github.com/Vincentive1232/Flying_Robot_Project.git
```

Then config and build the crazyflie-firmware:
```
cd Flying_Robot_Project/shared/crazyflie-firmware
make cf2_defconfig
make
```

Now we can choose a subproject and compile it. Take the **MEKF onboard estimator** as an example:
```
cd Estimator_MEKF
cd app_hello_rs_estimator
make
```

If your Crazyflie is ready to be flashed, follow the [instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) to flash the [OutofTreeEstimator](https://github.com/Vincentive1232/Flying_Robot_Project/tree/master/Geometric_Controller/app_hello_rs_math)/[OutofTreeController](https://github.com/Vincentive1232/Flying_Robot_Project/tree/master/Estimator_MEKF/app_hello_rs_estimator) to the hardware.