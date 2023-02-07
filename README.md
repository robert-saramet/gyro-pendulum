# gyro-pendulum
The system is composed of the pendulum itself and a computer program to save and display the collected data. The pendulum measures its own angle using an IMU unit and an advanced sensor fusion algorithm developed by NXP. The angle values are streamed in real time over UDP to the computer, which captures the data and exports it to a CSV file. The file is the opened by a second program, which generates a visual representation of the oscillations using matplotlib.

![Graph](https://raw.githubusercontent.com/robert-saramet/gyro-pendulum/main/pc/graph.png)
