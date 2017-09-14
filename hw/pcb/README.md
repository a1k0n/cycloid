This is a PCB which carries the Teensy 3.2 and connects to the servo & speed
controller on the car, as well as the stock R/C radio, wheel encoders, and an
analog feedback tap from the servo's internal potentiometer (requires modifying
servo).

It has the same mounting holes as the PCA9685 I2C PWM expander board commonly
used for the purpose of connecting a Raspberry Pi to an R/C car.

It can be seen (and purchased) here: 

<img src="https://644db4de3505c40a0444-327723bce298e3ff5813fb42baeefbaa.ssl.cf1.rackcdn.com/021b6d1d66606d016d28dd4cb05b3129.png" width="400"></img>
<br>
<a href="https://oshpark.com/shared_projects/DP0wzTcU">order on OSHPark</a>

(Note that I have a ton of extras; if you ask me at an Oakland DIYRobocars race
I'll give you one)

It also has an i2c piggyback connection for the IMU.

R1 and R2 are pull-up resistors for I2C; I do not recommend populating them,
however.

