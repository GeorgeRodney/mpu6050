MPU 6050 IMU class created.
Acceleration read out function created.
Gyroscope read out function created. 
Gyroscope calibration / offset function created. 


Build in debug mode:
    export PICO_SDK_PATH=../../pico-sdk
    cmake -DCMAKE_BUILD_TYPE=Debug ..

Connect to Pico:
    sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000"
    gdb-multiarch blink.elf
    target remote localhost:3333
    monitor reset init
    continue

# GDB RUN COMMANDS:
openDebugger
gdb accel.elf
> target remote localhost:3333
> monitor reset init
> continue