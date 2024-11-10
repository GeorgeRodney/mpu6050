MPU 6050 IMU class created.
Acceleration read out function created.
Gyroscope read out function created. 
Gyroscope calibration / offset function created. 


# GDB RUN COMMANDS:
openDebugger
gdb accel.elf
> target remote localhost:3333
> monitor reset init
> continue