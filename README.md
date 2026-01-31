This is all our code for the 2025-26 season DECODE.

## Download Instructions
Download the file as a .zip folder, unzip it on your computer and then open it in Android Studio <br>
Resync Gradle scripts once it opens (elephant icon in the top right) if some of the import statements show up as red on the screen <br>

## Panels Dashboard
This is the dashboard where we can see up-to-date telemtry info, we'll use this for tuning odometry and stuff <br>
Connect to the control hub wifi and then type in the IP address in your browser <br>
192.168.43.1:8001 <br> 

## Pedropathing Visualizer
You can use this to create your own paths and test them, it's been updated to the latest season's field so you can test routes for auto runs <br>
https://visualizer.pedropathing.com/ <br>

## Autonomous OpModes
All of our autonomous code is under the 'pedroPathing' module, and under 'auto'

## TeleOp Opmodes
All of our teleop code is under the 'teleop' module<br>
The demoTeleop (slower speed) and the robot-centric teleop opmodes have been disabled. If you want to use those opmodes, remove the @disabled statement and they will show up on the driver station<br>


## Sample OpModes
There are sample programs located under 'FtcRobotController' that can help you learn how to do specific things (ie. sounds or camera vision)
