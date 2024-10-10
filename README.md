Testing the BNO085 SlimyVR IMU Sensor The photo illustrates the connection setup. 

![BNO085_slimyVR](https://github.com/user-attachments/assets/74795e16-196d-4598-a7a1-642caf185d12)


The sensor communicates with the Arduino Mega via the I2C bus. When you pick up the BNO085 and rotate it left and right,
or tilt it sideways and forwards, you will observe a red box on the computer screen that mirrors these rotational movements.
Ensure that the baud rate is consistent in both the Python script and the Arduino program. The current baud rate is set at 9600. 

![adjustments](https://github.com/user-attachments/assets/4066bfc5-0454-4289-a9b4-0a5a742938f4)


The Arduino program, utilizing the Adafruit_BNO08x library, sends data to the COM12 port as follows:

Serial started. Trying to initialize BNO08x.
BNO08x Found!
X: 0.02 Y: -0.57 Z: -90.12
X: 0.03 Y: -0.57 Z: -90.12
X: 0.03 Y: -0.58 Z: -90.12
X: 0.03 Y: -0.57 Z: -90.12
X: 0.03 Y: -0.57 Z: -90.12
X: 0.03 Y: -0.56 Z: -90.12
X: 0.03 Y: -0.56 Z: -90.11
X: 0.03 Y: -0.56 Z: -90.11
X: 0.03 Y: -0.55 Z: -90.11
...

In this context, X, Y, and Z correspond to pitch, yaw, and roll respectively.


![Roll-Yaw-Pitch](https://github.com/user-attachments/assets/e5897706-58a4-42f2-becf-4cb44c5adb09)

The Python program extracts this data and adjusts the pitch, yaw, and roll angles of the box accordingly:

cuboid.rotation_x = -z % 360
cuboid.rotation_y = -x % 360
cuboid.rotation_z = y % 360

![BNO085-ursina](https://github.com/user-attachments/assets/84868254-cdcd-45ae-b4b9-7256a9c8ff2f)

Simultaneously, the program draws the coordinate system, marking the lines as follows: X = 1 line, Y = 2 lines, and Z = 3 lines.




