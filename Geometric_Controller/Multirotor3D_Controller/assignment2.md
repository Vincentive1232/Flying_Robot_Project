# Assignment 2 (Flying Robots, 2024/2025)

Implement the Lee geometric controller for the Bitcraze Crazyflie 2.1 robot. Test and tune in your simulator (Assignment 1). Execute physical test flights with your controller and report the tracking errors.

## Deliverables (by Dec. 20, 10am)

1. Your code.
    - Make sure to include your firmware (e.g., as a private git submodule that you add "whoenig" to)
2. A plot that shows the tracking errors (position, orientation, velocity, angular velocity) for the given figure8 trajectory over time.
    a) in simulation,
    b) on the physical platform.
3. The gains you used
    a) in simulation,
    b) on the physical platform.
4. A flight demonstration.

## Recommended Resources

- Class material (slides)
- [Example implementation and gains in C](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/controller/controller_lee.c)
- [bindgen docs](https://rust-lang.github.io/rust-bindgen/command-line-usage.html)
