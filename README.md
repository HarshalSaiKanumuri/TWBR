# Theory Of TWBR

1. Wiring

2. Measuring Angle with Acc

Accelerometer measure the rate of change in velocity ( Acceleration )
To measure the angle , the acceleration can be split into three or 2 acceleration components. A angle can be calculated as such :

3. Measuring Angle with Gyro

4. Offset ( not the rapper )

The offset is calculated by initially positioning the MPU-6050 in a vertical position , values are read over time of 100 millisec and an average is calculated.


5. Complimentary Function

Complimentary filter is used to combing the data from the gyroscope and accelerometer such that the effect to sensitivity to bias and drift are minimized.
This is done by producing a weighted average , where weights determine the level of influence of each sensor data. The weights are something that will be considered in the tuning of parameters stage.

6. How to generate an output ?

For this project I will be using a PID ( proportional-integral-derivative) controller where the output is limited to -255 to 255 ( this is the full range of PWM signal used to control the motors ).

7. 
