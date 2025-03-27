# Flying_Robot_Project
This repo is a primary and basic reimplementation of some of the functions in [Crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware/tree/master). 

## Project Structure
- `Dynamics`: 2D and 3D Dynamics Simulation for Crazyflie ("X" configuration).
  - `Multirotor2D`: Simulation code for a 2D Multirotor model.
  - `Multirotor3D`: Simulation code for a 3D Multirotor model with Euler Integration only.
  - `Multirotor3D_simple`: Simulation code for a 3D Multirotor model with Euler Integration and RK4 Integration.

- `Geometric_Controller`: Implementation of lee-controller in Rust, based on the Dynamics code.
  - `Multirotor3D_Controller`: Simulation code for lee-controller
  - `app_hello_rs_math`: Reimplementation of lee-controller in Rust using OutofTreeController api in Crazyflie-firmware.
  - `autonomous_sequence_high_level.py`: Guide the real drone to follow a figure-8 trajectory with rust lee-controller.

- `Estimator_MEKF`:
  - `StateEstimation_MEKF`: Simulation code for MEKF(Multiplicative Extended Kalman Filter)
  - `app_hello_rs_estimator`: Reimplementation of MEKF in Rust using OutofTreeEstimator api in Crazyflie-firmware.

- `Motion_Planning`:
  - `MP_Diff_Flat`: Simulation code for generating control input (thrust and torque) by using differential flatness.
  - `MP_Splines`: Generate reference trajectory by using polynomial splines and [cvxpy](https://github.com/cvxpy/cvxpy).

  
## Dependencies
### Setting Up Rust
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup update
```

### Meshcat
```
pip install meshcat
```
Then try the following command to check whether Meshcat is successfully installed:
```
meshcat-server --open
```


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

## Flash (only for real flight)
If your Crazyflie is ready to be flashed, follow the [instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md#using-crazyradio) to flash the [OutofTreeEstimator](https://github.com/Vincentive1232/Flying_Robot_Project/tree/master/Geometric_Controller/app_hello_rs_math) / [OutofTreeController](https://github.com/Vincentive1232/Flying_Robot_Project/tree/master/Estimator_MEKF/app_hello_rs_estimator) to the hardware.