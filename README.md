## Welcome!
This repository contains the official FTC SDK from https://github.com/FIRST-Tech-Challenge/FtcRobotController. It 
has the addition of our Titan Robotics Framework Library (TrcLib) and its dependencies. These are added as git 
submodules so that they can be independently managed without the need of changing the FTC SDK (except for minor 
gradle changes to tie in our Framework Library dependencies). Therefore, you can pick up the latest changes of 
our Framework Library without affecting the FTC SDK. It serves as a clean repository template for the start of a new season. This template also contains basic team code that implements a mecanum drive base with teleop control. It allows you to run a simple mecanum robot in TeleOp almost right out of the box. In addition, it includes autonomous infrastructure code that uses our FtcChoiceMenu for selecting autonomous strategies and other autonomous choice opotions. This allows you to write only one autonomous opmode to handle many permutations of autonomous. For example, one autonomous opmode can handle both red and blue alliances. The template also provides a rich set of Tests for diagnosing robot hardware and/or for tuning.

## Getting Started
To use this repository template, you can clone or fork this template repository to your own github repositories. There are many ways to do it but I am going to describe one way which is using GitHub Desktop. If you are more familiar with other similar tools, feel free to use it instead.
* On a web browser, enter the URL https://github.com/trc492/FtcTemplate.
* Then, click the "Fork" button near the upper right corner of the web page and answer the question on where you want to fork this repository to your GitHub repositories.
* Once the fork is done. Clone your GitHub repository to your computer using your favorite GitHub tool such as GitHub Desktop.
* On GitHub Desktop, click File->Clone repository..., select the repository you just forked and type in the path on the computer where you want to clone the repository to.

Congratulations! You just clone our template repository. Now you can fire up Android Studio and import this gradle project. Once that is done, you can now go to TeamCode and browse around the provided template code or sample code. You can compile the code and check if you have any issues with the cloned template. Or you can jump right in and start modifying/customizing the code.

### TeleOp Driving a Mecanum Robot Right Out Of The Box
Since this template already contains basic code for a mecanum robot base, it takes very few modifications to make it work with your mecanum robot.
* In RobotParams.java, update the string constants HWNAME_xxDRIVE_MOTOR corresponding to the hardware names of the four driving wheel motors in your robot config.
* Compile the code and deploy it to the robot.
* Place your robot on a stand so that the wheels can be free running without the robot running away from you. When looking down on the robot, the mecanum wheel rollers should form an X. If not, switch the mecanum wheels around until they form an X. On the Driver Station, activate your robot configuration. Select a TeleOp Opmode on the Driver Station called FtcTest. Press the init button to initialize the opmode. Press the D-pad down button on your driver gamepad until the "Test" shown on the Driver Station is on "Drive motors test". Press the D-pad right button to select the test. Then press the "Play" button on the Driver Station to start the opmode. This test will run each of the four driving wheels one after the other for 5 seconds each in the sequence of Left Front, Right Front, Left Back and Right Back. Note the rotation direction of each wheel and make sure they would have run the robot in the forward direction if the robot were placed on the ground. If any of the wheels are rotating in the wrong direction, correct them in RobotParams.java. Change xxDRIVE_INVERTED from true to false or vice versa to reverse the corresponding driving wheels until the test shows all four wheels rotating in the correct direction.

That's it. Your robot is now ready to be driven in TeleOp mode. The default drive mode is "Arcade Mode". It means the left stick on the driver gamepad controls the X and Y direction of the robot. The X-axis of the right stick controls the rotation. If you prefer, you can change the drive mode to "Holonomic Mode". In this mode, the Y-axis of the left stick controls the Y direction of the robot. The X-axis of the right stick controls the X direction, the left trigger controls turning left and the right trigger controls turning right. To change to this mode, change ROBOT_DRIVE_MODE in RobotParams.java to DriveMode.HOLONOMIC_MODE.

In addition, there are a few more buttons on the driver gamepad that modify how the robot is driven. Click the right bumper on the driver gamepad will toggle between Robot and Field Oriented driving modes. Our library also supports Inverted driving mode allowing the robot to switch the front and back end. This is useful for a robot that has an end effector such as an intake at the back so that the drivers can drive the robot around as if the intake is in front. Press and hold the left bumper allows you to drive the robot at slow speed. This is useful for delicate movement of the robot.

### Making Drive Base Odometry Work
Our library supports both drive wheel motor odometry (using drive wheel motor encoders) and passive wheel odometry (aka dead-wheel odometry or odo pods). To select which odometry to use, change RobotParams.Preferences.useExternalOdometry to true to use passive wheel odometry or false to use drive wheel motor odometry. When using odo pods, you need to provide the odo pod placement info relative to the robot's centroid (*_ODWHEEL_*_OFFSET in RobotParams). To determine the robot's centroid, draw a rectangle with each drive wheel being the corners of the rectangle. The centroid is the center point of this rectangle. It is very important to provide accurate distance offsets of each odo pod wheel to the robot centroid because this affects the accuracy of the odometry calculation. We recommend measuring the offset distances from the CAD model if possible. Measuring the offset distances by hand introduces a lot of error and therefore not recommended. Our library uses ENU (East-North-Up) coordinate system for the robot which means robot centroid on the ground is the origin with X-axis pointing to robot right, Y-axis pointing to robot forward and Z-axis pointing up. Therefore the left odo pod will have a negative x-offset from robot centroid and the right odo pod will have a positive x-offset from robot centroid and so on. Even though our library supports 2 to 4 odo pods, the template code assumes you are using 3 odo pods, 2 pointing forward, typically one on the left and one on the right (Y-axis) and one pointing sideway (X-axis). Our library uses gyro (IMU) to keep track of the robot heading. In theory, it could use the odo pods to calculate the robot's heading but gyro is much more accurate than using odo pods.

The next step is to determine the odometry scales (X and Y). This is applicable for both drive wheel motor odometry and odo pods. Generally, encoders give you values in the unit of counts (or ticks). To be more useful, we would like the odometry values to be in real world units such as inches or meters. Our library will scale the odometry to the unit of your choice. There are two ways to determine the odometry scales, one is to calculate it by providing info such as odo wheel diameter, encoder CPR (Count-Per-Revolution) etc. See RobotParams.ODO_WHEEL_* and RobotParams.*POS_INCHES_PER_COUNT. Another way is to calibrate the scales empiracally. This can be done by first setting the X and Y scales to 1 so that the reported units are unscaled and therefore in the unit of encoder counts. Then, reset all the encoders and manually drive the robot in either the X or Y direction for a distance when calibrating X or Y scales. Measure the distance traveled. Then scale = distanceTraveled / unscaledCount. To minimize calibration error, drive a longer distance (at least 6 to 8 feet). Then update the *_INCHES_PER_COUNT variables with the calculated scale. Repeat the calibration again to check if the reading is within some tolerance of the actual measurement. If not, calculate the new scale by newScale = oldScale * actualValue / reportedValue. Repeat this process until the reported value is within some tolerance of the actual measurement. Once you have the odometry scales calibrated, you should be able to drive the robot around and odometry will keep track of the robot location on the field relative to its start location.

### Creating Subsystems
Once the drive base is fully functional, the next step is to create subsystems for the robot such as Elevator, Arm, Intake, Grabber etc. It is a good practice to create subsystems as separate Java classes that encapsulate all hardware related to those subsystems. To create a subsystem, follow the steps below:
1. Create a Java class in the subsystems folder (e.g. Slide.java).
   ```
   public class Slide
   {
       private final TrcMotor slideMotor;

       /**
        * Constructor: Creates an instance of the object.
        */
       public Slide()
       {
           FtcMotorActuator.Params slideParams = new FtcMotorActuator.Params()
               .setMotorInverted(RobotParams.SLIDE_MOTOR_INVERTED)
               .setLowerLimitSwitch(
                   RobotParams.SLIDE_HAS_LOWER_LIMIT_SWITCH,
                   RobotParams.SLIDE_LOWER_LIMIT_INVERTED)
               .setUpperLimitSwitch(
                   RobotParams.SLIDE_HAS_UPPER_LIMIT_SWITCH,
                   RobotParams.SLIDE_UPPER_LIMIT_INVERTED)
               .setVoltageCompensationEnabled(RobotParams.SLIDE_VOLTAGE_COMP_ENABLED)
               .setPositionScaleAndOffset(RobotParams.SLIDE_INCHES_PER_COUNT, RobotParams.SLIDE_OFFSET)
               .setPositionPresets(RobotParams.SLIDE_PRESET_TOLERANCE, RobotParams.SLIDE_PRESETS);
           slideMotor = new FtcMotorActuator(RobotParams.HWNAME_SLIDE, slideParams).getActuator();
           slideMotor.setSoftwarePidEnabled(true);
           slideMotor.setPositionPidCoefficients(
               RobotParams.SLIDE_KP, RobotParams.SLIDE_KI, RobotParams.SLIDE_KD, RobotParams.SLIDE_KF,
               RobotParams.SLIDE_IZONE);
           slideMotor.setPositionPidTolerance(RobotParams.SLIDE_TOLERANCE);
           slideMotor.setStallDetectionEnabled(
               RobotParams.SLIDE_STALL_DETECTION_DELAY, RobotParams.SLIDE_STALL_DETECTION_TIMEOUT,
               RobotParams.SLIDE_STALL_ERR_RATE_THRESHOLD);
       }

       public TrcMotor getSlideMotor()
       {
           return slideMotor;
       }
   }
   ```
2. In Robot.java, add a public class variable in the Subsystem section.
   ```
   public TrcMotor slide;
   ```
3. In the constructor of Robot.java, under the Subsystem section, add code to create and initialize the subsystem.
   ```
   if (RobotParams.Preferences.useSlide)
   {
       slide = new Slide().getSlideMotor();
   }
   ```
4. In RobotParams.java, add a Preferences that can turn the subsystem ON or OFF. This is very useful when developing code for an unfinished robot where some subsystems may or may not exist yet.
   ```
   public static boolean useSlide = true;
   ```
5. In RobotParams.java, add a HWNAME for the subsystem. This will become either the name or name prefix for the hardware in the robot configuration (e.g. slide.motor, slide.lowerLimitSw etc).
   ```
   public static final String HWNAME_SLIDE = "slide";
   ```
6. At the end of RobotParams.java, under the Subsystem section, add constants that characterize the subsystem.
   ```
   public static final boolean SLIDE_MOTOR_INVERTED = false;
   public static final boolean SLIDE_HAS_LOWER_LIMIT_SWITCH = true;
   public static final boolean SLIDE_LOWER_LIMIT_INVERTED = false;
   public static final boolean SLIDE_HAS_UPPER_LIMIT_SWITCH = false;
   public static final boolean SLIDE_UPPER_LIMIT_INVERTED = false;
   public static final boolean SLIDE_VOLTAGE_COMP_ENABLED = true;
   public static final double SLIDE_ENCODER_PPR = GOBILDA_5203_312_ENCODER_PPR;
   public static final double SLIDE_PULLEY_DIAMETER = 1.405;
   public static final double SLIDE_PULLEY_CIRCUMFERENCE = Math.PI*SLIDE_PULLEY_DIAMETER;
   public static final double SLIDE_INCHES_PER_COUNT = SLIDE_PULLEY_CIRCUMFERENCE/SLIDE_ENCODER_PPR;
   public static final double SLIDE_POWER_LIMIT = 1.0;
   public static final double SLIDE_OFFSET = 0.0;
   public static final double SLIDE_MIN_POS = SLIDE_OFFSET;
   public static final double SLIDE_MAX_POS = 24.0;
   public static final double SLIDE_POS_1 = 6.0;
   public static final double SLIDE_POS_2 = 12.0;
   public static final double SLIDE_POS_3 = 18.0;
   // Power settings.
   public static final double SLIDE_CAL_POWER = -0.25;
   // Preset positions.
   public static final double SLIDE_PRESET_TOLERANCE = 0.5;
   public static final double[] SLIDE_PRESETS = new double[] {
       SLIDE_MIN_POS, SLIDE_POS_1, SLIDE_POS_2, SLIDE_POS_3, SLIDE_MAX_POS
   };
   // PID Actuator parameters.
   public static final double SLIDE_KP = 0.6;
   public static final double SLIDE_KI = 0.0;
   public static final double SLIDE_KD = 0.025;
   public static final double SLIDE_KF = 0.0;
   public static final double SLIDE_TOLERANCE = 0.5;
   public static final double SLIDE_IZONE = 10.0;
   public static final double SLIDE_STALL_DETECTION_DELAY = 0.5;
   public static final double SLIDE_STALL_DETECTION_TIMEOUT = 0.2;
   public static final double SLIDE_STALL_ERR_RATE_THRESHOLD = 5.0;
   ```
7. In Robot.java, add code to the method updateStatus to display the status of the subsystem on the Driver Station. This is very useful for debugging. For example, if the subsystem is not working, you can look at the subsystem status to figure out if the problem is in the code, wiring or hardware.
   ```
   if (slide != null)
   {
       dashboard.displayPrintf(
           lineNum++,
           "Slide: power=" + slide.getPower() +
           ", pos=" + slide.getPosition() + "/" + slide.getPidTarget() +
           ", lowerLimit=" + slide.isLowerLimitSwitchActive());
   }
   ```
8. In FtcTeleOp.java, add a private class variable slidePrevPower and add code to the method periodic to read analog controls on the gamepad for controlling the subsystem.
   ```
   private double slidePrevPower = 0.0;
   ...
   if (robot.slide != null)
   {
       double slidePower = operatorGamepad.getLeftStickY(true) * RobotParams.SLIDE_POWER_LIMIT;
       if (slidePower != slidePrevPower)
       {
           if (manualOverride)
           {
               // Not using PID control.
               robot.slide.setPower(slidePower);
           }
           else
           {
               // Using PID control.
               robot.slide.setPidPower(
                   slidePower, RobotParams.SLIDE_MIN_POS, RobotParams.SLIDE_MAX_POS, true);
           }
           slidePrevPower = slidePower;
       }
   }
   ```
9. In FtcTeleOp.java, add code to the method operatorButtonEvent to read button controls on the gamepad for controlling the subsystem. The following example shows how you would control the slide to extend or retract to the next preset position by using the DPad on the operator gamepad as well as using the BACK button to zero calibrate the slide.
   ```
   case FtcGamepad.GAMEPAD_DPAD_UP:
       if (pressed && robot.slide != null)
       {
           robot.globalTracer.traceInfo(moduleName, ">>>>> Extending slide to next position.");
           robot.slide.presetPositionUp(null, RobotParams.SLIDE_POWER_LIMIT);
       }
       break;

   case FtcGamepad.GAMEPAD_DPAD_DOWN:
       if (pressed && robot.slide != null)
       {
           robot.globalTracer.traceInfo(moduleName, ">>>>> Retracting slide to the next position.");
           robot.slide.presetPositionDown(null, RobotParams.SLIDE_POWER_LIMIT);
       }
       break;

   case FtcGamepad.GAMEPAD_BACK:
       if (pressed && robot.slide != null)
       {
           robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate.");
           robot.slide.zeroCalibrate(RobotParams.SLIDE_CAL_POWER);
       }
       break;
   ```
Once the subsystem is created and tied in to TeleOp, you should be able to operate the subsystem in TeleOp mode and check out the subsystem status on the Driver Station.

## TRC Framework Library Features
Our Framework Library provides numerous features. We will list some of them here:
- FtcOpMode: Our own opmode that extends LinearOpMode but providing interface similar to OpMode where you put your code in some sort of loop method. FtcOpMode is a cooperative multi-tasking scheduler. As an advanced feature, our Framework Library also supports multi-threaded true multi-tasking. But for rookie teams who don't want to tackle the gotchas of true multi-tasking, cooperative multi-tasking is the way to go. This allows your autonomous to operate multiple subsystems at the same time instead of doing things sequentially. This is especially important since FTC autonomous period lasts only 30 seconds. In order to perform the maximum number of tasks in the autonomous period, your code would want to perform multiple tasks that have no dependencies on each other and perform them simultaneously. The Framework Library enables that in a trivial manor.
- State machine: The state machine infrastructure is the core of multi-tasking. Each task should use a state machine to keep track of their states. This allows FtcOpMode to switch between tasks and be able to maintain the state of each task when they are resumed from suspended state.
- Task syncrhonization: Some tasks have dependencies on each other. For example, autonomous may want to finish driving the robot to the specified location before dumping the game element to the proper spot. This requires task synchronization. The Framework Library provides a number of task synchronization features such as Events (TrcEvent) and Callbacks (TrcEvent.Callback). Event allows an operation to signal it when the operation is completed so that the task waiting for it can resume. Callback allows a method to be called to perform additional work without the use of a state machine after the operation is completed, for example.
- Timers: The Timer Manager (TrcTimer) manages multiple simultaneous timers. When a timer expires, you have the option of signaling an event or doing an event callback. For example, if you want to spin a motor for 3 seconds and turn it off afterwards, you can arm a timer that expires in 3 seconds and do an event callback to turn the motor off. This type of operation is sometimes called "fire and forget".
- Advanced multi-tasking: In addition to cooperative multi-task, our Framework Library also supports multi-threaded tasks. The Library provides a number of standard threads (i.e. main robot thread and IO thread). The main robot thread runs the FtcOpMode where the scheduler is performing cooperative multi-tasking on the main robot thread. The IO thread handles all input tasks such as reading sensors and odometry and output tasks such as motor and actuator tasks including PID control and pathing. If there is a special need that either requires high frequency processing and cannot afford any latency or a task that takes extra long time to run and thus blocking the thread unnecessarily long, the Library enables you to create STANDALONE tasks that have their own thread. All these tasks/threads are mananged by the Task Manager (TrcTaskMgr). Although everything provided and maintained by the Library are thread-safe, user must still be cautious when writing multi-threading code. Care must be taken to ensure you don't fall into the trap of two common multi-threaded programming woes: shared resource contention and task synchronization. If you don't understand these, it's better not to do multi-threaded multi-tasking and stick only with cooperative multi-tasking. Even with cooperative multi-tasking, you still need to apply some simple disciplines: do not block the main robot thread (i.e. no busy wait loop and sleep statements in your task code). Just start an operation and get out. Do not wait for it to complete. Most of the operations supported by the Framework Library are asynchronous. For a subsystem to support asynchronous operation, it must provide a way to start the operation without blocking and a way to signal/notify the completion of the operation. Calling them will start the operation and the control is returned back to your code immediately. Your state machine should take care waiting for an operation to complete. That's why it's called "cooperative multi-tasking". You must be good citizens for it to work properly.
- Inputs: The Framework Library supports many different input devices such as gamepad controllers, sensors and driver station dashboard.
  - Gamepad controller: The Framework Library monitor all buttons on the gamepad for state changes. Any button presses or releases will result in an event callback to your button event handler. This simplifies your TeleOp code tremendously.
  - Sensors: The Framework Library supports many type of sensors whether they are digital sensors, analog sensors, I2C sensors or even Android built-in sensors, the Library provides access to them. Popular sensors include ultronic sensor, color sensor, distance sensor such as Lidar, gyro, accelerometer, touch sensor, IR seeker and Pixy camera etc. It also supports many underlying communication protocols (e.g. I2C, Serial, SPI etc) so that you can write custom sensor driver code to communicate with sensors that the Library does not have built-in support.
  - Driver station dashboard: The Framework Library provides easy access to the driver station as an input device. It allows you to create Choice Menus (FtcChoiceMenu) or Value Menus (FtcValueMenus) where you can ask the user to provide information before the competition match is started. Information such as whether you are on the RED or BLUE Alliance; which starting position your robot is in; whether your robot should delay starting the autonomous routine to let your alliance partner to go first to avoid potential collision or whether your robot should perform or skip a certain autonomous tasks. The Choice and Value menus form a decision tree that allow the user to select choices or enter values using the gamepad buttons.
- Data Filters and Processors: In the real world, sensors have noises. In some applications, noises are bad for robot control. The Framework Library provides a number of popular data filters (e.g. IIR, Kalman and Spurious filters etc) that will clean up noises on sensor readings. It also provides some data converters such as data integrator. For example, some simple gyro sensors only give you rotational rate but not heading. You must integrate the rotational rate over time to calculate heading. In this situation, the Library provides the data integrator. In some other sensors such as some gyros or compasses, they give you non-contiguous readings when passing through some point such as the REV IMU gyro goes from 179 degrees to -180, or compass goes from 359 degrees back to zero. This non-contiguous values may cause havoc in control algorithms. In this case, the Library provides a converter that can monitor the sensor crossing such points and convert the values into a contiguous scale.
- Outputs: The Framework Library supports many types of output devices such as motors, servo, complex actuators, lights and sound.
  - Motors: Motor is the most fundamental output device on a robot. It provides movements for the robot. FTC SDK provides some basic motor classes (e.g. DcMotor, DcMotorEx). The Framework Library adds a lot more features on top of that in different layers of complexity. For example, it provides FtcDcMotor extending TrcMotor that added support for a digital input sensor to reset the motor encoder automatically (limit switches). This is useful for using the motor in a complex actuator such as an arm or elevator when you may need to zero calibrate the zero position of the actuator using a lower limit switch. It also added support to provide velocity mode control and motor odometry. On top of the fundamental motor features, it also provided PID Controlled functionality. It can support either native motor close-loop control (position and velocity PID control) or software PID control in case some motors do not support native close-loop control (e.g. continuous servos). TrcMotor added support for lower and upper limit switches, motor stall protection for safety, multiple motors with synchronization (motor followers), zero position calibration and gravity compensation. These advanced features made it trivial to implement complex subsystems such as a swing arm or elevator. The built-in PIDF controller allows the arm or elevator to be controlled by an analog joystick to speed up or slow down the arm/elevator movement. It understands the arm/elevator position approaching the lower/upper position limits and will automatically slow down its movement. It also provides stall protection. If the PID Actuator got stuck and the motor is stalled, it can cut motor power to prevent it from burning out. It also allows a reset timeout so that the stall condition can be cleared after a certain period assuming the stall condition is caused by human temporarily. This allows the subsystem to resume its function and provides time for the motor to cool down. In addition, it also supports voltage compensation. It understands battery voltage drop and can compensate the power level sent to the motor.
  - Servos: With the limited number of motors allowed by FTC, servo motors become the secondary important actuator on a robot. The Framework Library provides the basic support of a servo over the FTC SDK (TrcServo). It supports translation between logical servo positions (between the value of 0 and 1) to physical positions such as 0 to 180 degrees. Just like motors, it also allows you to invert the direction of the servo movement. In addition, it also provides features to support multiple servos (followers), continuous servo with optional encoder, lower and upper limit switches. Most importantly, it allows speed controlling a servo motor so you can control a servo by an analog joystick.
  - Light: The Framework Library supports many ways to control lights, usually LEDs. They could be a single LED with just one color, a single RGB LED or a RGB LED strip that is pixel addressable. The LED lights can be controlled by digital output ports or most likely the REV Blinkin LED controller. This is not only for asethetics to make our robot look pretty, it also serves a very practical purpose: providing feedback to drivers on robot status. For example, it can tell you whether robot vision detected the target by changing the LED color on different target positions. The Library not only allows you to lit up LEDs in different colors or different color patterns, it also provides a complex priority scheme in controlling the LEDs. Imagine the robot has many subsystems that want to tell you something. Vision may want to tell you whether it sees the target, intake may want to tell you whether it has taken in a game piece. All these events will cause LED contention (i.e. different subsystems fighting to change the color of the LEDs to show their status). The Library provides a priority scheme (TrcPriorityIndicator) that defines what color pattern has priority over the others so that important events can override lower priority events to show their status. The Library also provides support for LED Matrix Panel. Although it is not really practical for FTC, it is more an FRC feature because of power requirement and also requires custom electronics that FTC may not allow.
  - Sound: The Framework Library supports sound. It used to support sound on the Robot Controller when it was an Android Phone. With the introduction of REV Robot Controller Hub, it no longer provides sound capability. The sound support now gets redirected to the Driver Station. Sound support includes playing a tone or providing text to speech. Sound is an important output device. Just like light, it provides feedback to the drivers on important robot status. For example, if one of the motors is stalled, the Library can cut power to the motor to prevent it from burning out. Sound support can generate a beep to warn the driver about it.
- Drive Base: Our Framework Library provided support for 3 different types of drive bases: Simple Differential Drive Base (TrcSimpleDriveBase), Mecanum Drive Base (TrcMecanumDriveBase) and Swerve Drive Base (TrcSwerveDriveBase). All drive bases have built-in kinematics, odometry and localization support. It means the library can use sensors such as wheel encoders and gyro to keep track of the absolute field location of the robot. It can combine the sensor readings to control the movement of the robot by applying kinematic calculations. It even supports passive-wheel odometry (aka dead-wheel odometry) where it supports 2 to 4 passive omniwheels with encoders to keep track of the absolute field location of the robot. All odometry data can be scaled to real world unit such as inches instead of encoder counts. All drive bases support many advanced features such as stall detection that detects if the drive base is stalled because it runs into an obstacle. This allows the possibility of writing advanced code such as running into the field wall to relocalize the robot location. Running into the field wall causes the drive base to stall which can signal an event to stop the drive base and reset the robot location to a known position. All drive bases provide support for different drive strategies such as tankDrive, arcadeDrive and curveDrive. It also provides support for holonomicDrive for drive bases that have this capability such as Mecanum and Swerve Drive Bases. For holonomic capable drive bases, it also supports field oriented driving where the robot can go in any direction independent of the robot's heading. It also supports Gyro-Assist driving where it uses the gyro to keep the robot driving straight. If the robot has a weight distribution problem or one of the wheels has more friction than the others, the robot won't drive straight. The robot may curve to the left or right. By enabling Gyro-Assist driving, it will use the gyro to maintain a straight heading.
  - Simple Differential Drive Base supports drive bases with 2 to 6 motors. It has left and right sides. Each side can have 1 to 3 motors. Simple Drive Base can only run straight and cannot strafe like Mecanum Drive Bases (i.e. no holonomicDrive support, only differential drive).
  - Mecanum Drive Base has 4 mecanum wheels. Each wheel has its own motor. It is capable of holonomic drive (i.e. strafing).
  - Swerve Drive Base has 4 swerve wheel modules. Each swerve module consists of a drive motor and a steering motor. The steering motor can be a DC motor or a continuous servo motor. Each swerve module on a Swerve Drive Base can be independently steer so that it can run in any direction with the robot heading pointing to a totally different direction.
- Exclusive Subsystem: A robot consists of many subsystems (e.g. drive base, elevator, shooter, intake, vision etc). Most of the subsystems can be operated by human operator in TeleOp mode. However, some subsystems can also be used in auto-assist operations. For example, on the press of a button, the vision-assisted shooter may stop the drive base, acquire a camera image for vision processing, calculate the trajectory for shooting a target, spin the shooter up to shooting speed, pan and tilt the shooter to the correct angles aiming at the target and shoot. The auto-assist operation involves several subsystems while these subsystems can also be operated by human control. Without coordination, human control and auto-assist may fight each other for the control of these subsystems. For example, while auto-assist is tilting the shooter for aiming the target, the tilting mechanism is also controlled by a joystick which is at neutral position. It means teleop code is constantly sending zero power to the tilter while auto-assist is trying to move the tilter. This causes jerky movement of the tilter. With Exclusive Subsystem support, one can declare the tilter as an Exclusive Subsystem. Before auto-assist starts an operation with the Exclusive Subsystem, it must acquire exclusive ownership of the subsystem. Once ownership is acquired, nobody else except the owner can operate the subsystem. This prevents teleop control from interfering with the auto-assist operation. When the auto-assist operation is done, the exclusive ownership of the subsystems will be released so that telop control can be resumed.
- PIDF Control: Aside from supporting motor native close-loop control, our framework library also supports software PIDF control. Our software PIDF controller has a lot of extra features such as iZone in addition to P.I.D.F. It also understands target tolerance, stalling due to steady state error. It understands absolute versus relative target setpoint. It supports options to disallow oscillation (i.e. if the controller overshoots, it will just stop even though it exceeded tolerance). It also supports setting close-loop ramp rate and stall detection for aborting close-loop control hang (i.e. steady state error exceeding tolerance causing close-loop control to wait forever). It also allows setting close-loop control maximum power limits.
- Motor Odometry: Motor odometry keeps track of motor position as well as velocity in real world units (e.g. inches) instead of encoder counts. If the motor does not support native velocity report, our library will calculate the velocity for you. Motor odometry is highly optimized for performance. We support Lynx Bulk Caching mode that guarantee reading motor odometry only once in a robot loop. Our code has built-in performance monitors for debugging performance issues.
- Drive Base Odometry: Our drive base odometry supports both drive motor odometry as well as passive wheel odometry (Passive Odometry Pods). With drive wheel odometry, it reads either drive motor odometry or passive wheel odometry as well as gyro heading to calculate the absolute field location of the drive base.
- Pathing Following: Our library supports Pure Pursuit Drive path following. In addition to the ability of following a pre-planned path, it also allows dynamically generated path. This is very useful for working with vision system that detects target location and can dynamically create a path navigating the robot to the detected target.
- Vision: In every season, the game play usually involves navigating the robot to a certain location marked by objects detectable by vision or the robot is shooting game elements at a target recognizable by vision. Computer vision can have different complexity, ranging from simple color blob detection to full blown neural network machine learning object recognition. Therefore, it is generally computational intensive and could take hundreds of milliseconds to process an image frame. Fortunately, there are industrial vision libraries that take care of the heavy lifting for us. Libraries such as Tensor Flow, AprilTag and OpenCV. Our Framework Library includes support for all these industrial libraries. Some industrial libraries support asynchronous processing of image frames and some do not. For those that don't, they could block our main robot thread for hundreds of milliseconds. But with our Framework Library, we wrap these industrial libraries with easy to use interfaces and provide asynchronous support thus freeing our main robot thread to take care of other tasks. This encapsulation makes it extremely easy to writing vision code.
- Util:
  - Trace logging: In addition of providing information output to the dashboard on the driver station, the Framework Library also provides trace logging mainly for debugging purpose. It is a super important tool allowing post-mortem analysis of the robot performance of a match either for debugging or for performance tuning. Information written to the trace log has different levels that can be adjusted to reduce clutter. The levels are: VERBOSE, DEBUG, INFO, WARNING, ERROR and FATAL. For example, in regular competition match log, we will only turn up to the INFO level at the most. But for debugging, we may turn up to DEBUG or even VERBOSE to see everything.

## Getting Help
### User Documentation and Tutorials
We are not very good at creating documentation and tutorials but we want to get better at this. Our Framework Library code has JavaDoc all over. Therefore, you can get information on what each class does and their methods. We have also added sample code to the template project. Several teams have been using our Framework Library and we welcome opportunities of collaboration in creating tutorial materials. Feel free to suggest what tutorial you want to see.

### Javadoc Reference Material
The Javadoc reference documentation for the TRC Robotics Framework Library can be found [here](https://trc492.github.io/FtcJavaDoc/). We have also created some class material for learning our Robotics Framework [here](https://github.com/trc492/TrcAdvancedProgrammingClass/tree/main/FtcLessons).

### Online User Forum
For technical questions regarding our Framework Library, please post questions on the FTC Forums [here](https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio).

### Sample OpModes
In addition, we provide a large selection of sample OpModes (sample robot code) that show you how to use various features of our library. You can find them in a separate GitHub repository [here](https://github.com/trc492/TrcFtcSamples).

# FTC SDK Release Information

## NOTICE

This repository contains the public FTC SDK for the CENTERSTAGE (2023-2024) competition season.

## Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

## Requirements
To use this Android Studio project, you will need Android Studio 2021.2 (codename Chipmunk) or later.

To program your robot in Blocks or OnBot Java, you do not need Android Studio.

## Getting Started
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html), and then migrate to the [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) or to [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html) afterwards.

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo.  Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the [Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) page for this repository.

* The Releases page also contains prebuilt APKs.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
In addition, we provide a large selection of Sample OpModes (robot code examples) that show you how to use various features of our library. You can find them in a separate GitHub repository [here](https://github.com/trc492/TrcFtcSamples)

# Release Information

## Version 9.1 (20240215-115542)

### Enhancements
* Fixes a problem with Blocks: if the user closes a Block's warning balloon, it will still be closed next time the project is opened in the Blocks editor.
* In the Blocks editor, an alert concerning missing hardware devices is not shown if all the Blocks that use the missing hardware devices are disabled.
* Adds Blocks to support comparing property values CRServo.Direction, DCMotor.Direction, DCMotor.Mode, DCMotor.ZeroPowerBehavior, DigitalChannel.Mode, GyroSensor.HeadingMode, IrSeekerSensor.Mode, and Servo.Direction, to the corresponding enum Block.
* Improves OnBotJava auto-import to correctly import classes when used in certain situations.
* Improves OnBotJava autocomplete to provide better completion options in most cases.
  * This fixes an issue where autocomplete would fail if a method with two or more formal parameters was defined.
* In OnBotJava, code folding support was added to expand and collapse code sections
* In OnBotJava, the copyright header is now automatically collapsed loading new files
* For all Blocks OpMode samples, intro comments have been moved to the RunOpMode comment balloon.
* The Clean up Blocks command in the Blocks editor now positions function Blocks so their comment balloons don't overlap other function Blocks.
* Added Blocks OpMode sample SensorTouch.
* Added Java OpMode sample SensorDigitalTouch.
* Several improvements to VisionPortal
  * Adds option to control whether the stream is automatically started following a `.build()` call on a VisionPortal Builder
  * Adds option to control whether the vision processing statistics overlay is rendered or not
  * VisionPortals now implement the `CameraStreamSource` interface, allowing multiportal users to select which portal is routed to the DS in INIT by calling CameraStreamServer.getInstance().setSource(visionPortal). Can be selected via gamepad, between Camera Stream sessions.
  * Add option to `AprilTagProcessor` to suppress calibration warnings
  * Improves camera calibration warnings
    * If a calibration is scaled, the resolution it was scaled from will be listed
    * If calibrations exist with the wrong aspect ratio, the calibrated resolutions will be listed
  * Fixes race condition which caused app crash when calling `stopStreaming()` immediately followed by `close()` on a VisionPortal
  * Fixes IllegalStateException when calling `stopStreaming()` immediately after building a VisionPortal
  * Added FTC Blocks counterparts to new Java methods:
    * VisionPortal.Builder.setAutoStartStreamOnBuild
    * VisionPortal.Builder.setShowStatsOverlay
    * AprilTagProcessor.Builder.setSuppressCalibrationWarnings
    * CameraStreamServer.setSourceâ€‹

### Bug Fixes
* Fixes a problem where OnBotJava does not apply font size settings to the editor.
* Updates EasyOpenCV dependency to v1.7.1
  * Fixes inability to use EasyOpenCV CameraFactory in OnBotJava
  * Fixes entire RC app crash when user pipeline throws an exception
  * Fixes entire RC app crash when user user canvas annotator throws an exception
  * Use the modern stacktrace display when handling user exceptions instead of the legacy ESTOP telemetry message

## Version 9.0.1 (20230929-083754)

### Enhancements
* Updates AprilTag samples to include Decimation and additional Comments.  Also corrects misleading tag ID warnings
* Increases maximum size of Blocks inline comments to 140 characters
* Adds Blocks sample BasicOmniOpMode.
* Updated CENTERSTAGE library AprilTag orientation quaternions
    * Thanks [@FromenActual](https://github.com/FromenActual)
* Updated Java Sample ConceptTensorFlowObjectDetection.java to include missing elements needed for custom model support.

### Bug Fixes
* Fixes a problem where after October 1 the Driver Station will report as obsolete on v9.0 and prompt the user to update.

## Version 9.0 (20230830-154348)

### Breaking Changes
* Removes Vuforia
* Fields in `AprilTagDetection` and `AprilTagPose(ftc/raw)` objects are now `final`
* VisionPortal builder method `setCameraMonitorViewId()` has been renamed to `setLiveViewContainerId()` and `enableCameraMonitoring()` has been renamed to `enableLiveView()`

### Enhancements
* Adds support for the DFRobot HuskyLens Vision Sensor.
* Blocks teams can now perform webcam calibration.
    * Added a Block for System.currentTimeMillis (under Utilities/Time)
    * Added a Block for VisionPortal.saveNextFrameRaw (under Vision/VisionPortal)
    * Added a new sample Blocks OpMode called UtilityCameraFrameCapture.
* The RobotDriveByGyro sample has been updated to use the new universal IMU interface.  It now supports both IMU types.
* Removed some error-prone ElapsedTime Blocks from the Blocks editor's toolbox. This is not a
  breaking change: old Blocks OpModes that use these Blocks will still function, both in the
  Blocks editor and at runtime.
* Standardizes on the form "OpMode" for the term OpMode.
    * The preferred way to refer to OpModes that specifically extend `LinearOpMode` (including Blocks OpModes) is "linear OpMode".
    * The preferred way to refer to OpModes that specifically extend `OpMode` directly is "iterative OpMode".
* Overhauls `OpMode` and `LinearOpMode` Javadoc comments to be easier to read and include more detail.
* Makes minor enhancements to Java samples
    * Javadoc comments in samples that could be rendered badly in Android Studio have been converted to standard multi-line comments
    * Consistency between samples has been improved
    * The SensorDigitalTouch sample has been replaced with a new SensorTouch sample that uses the `TouchSensor` interface instead of `DigitalChannel`.
    * The ConceptCompassCalibration, SensorMRCompass, and SensorMRIRSeeker samples have been deleted, as they are not useful for modern FTC competitions.

### Bug Fixes
* Fixes a bug which prevented PlayStation gamepads from being used in bluetooth mode. Bluetooth is NOT legal for competition but may be useful to allow a DS device to be used while charging, or at an outreach event.
* Fixes a bug where a Blocks OpMode's Date Modified value can change to December 31, 1969, if the Control Hub is rebooted while the Blocks OpMode is being edited.
* Fixes the automatic TeleOp preselection feature (was broken in 8.2)
* Fixes a bug where passing an integer number such as 123 to the Telemetry.addData block that takes a number shows up as 123.0 in the telemetry.
* Fixes OnBotJava autocomplete issues:
  * Autocomplete would incorrectly provide values for the current class when autocompleting a local variable
  * `hardwareMap` autocomplete would incorrectly include lambda class entries
* Fixes OnBotJava not automatically importing classes.
* Fixes OnBotJava tabs failing to close when their file is deleted.
* Fixes a project view refresh not happening when a file is renamed in OnBotJava.
* Fixes the "Download" context menu item for external libraries in the OnBotJava interface.
* Fixes issue where Driver Station telemetry would intermittently freeze when set to Monospace mode.
* Fixes performance regression for certain REV Hub operations that was introduced in version 8.2.
* Fixes TagID comparison logic in DriveToTag samples.

## Version 8.2 (20230707-131020)

### Breaking Changes
* Non-linear (iterative) OpModes are no longer allowed to manipulate actuators in their `stop()` method. Attempts to do so will be ignored and logged.
  * When an OpMode attempts to illegally manipulate an actuator, the Robot Controller will print a log message
    including the text `CANCELLED_FOR_SAFETY`.
  * Additionally, LinearOpModes are no longer able to regain the ability to manipulate actuators by removing their
    thread's interrupt or using another thread.
* Removes support for Android version 6.0 (Marshmallow). The minSdkVersion is now 24.
* Increases the Robocol version.
  * This means an 8.2 or later Robot Controller or Driver Station will not be able to communicate with an 8.1 or earlier Driver Station or Robot Controller.
  * If you forget to update both apps at the same time, an error message will be shown explaining which app is older and should be updated.
* FTC_FieldCoordinateSystemDefinition.pdf has been moved.  It is still in the git history, but has been removed from the git snapshot corresponding with the 8.2 tag.  The official version now lives at [Field Coordinate System](https://ftc-docs.firstinspires.org/field-coordinate-system).
* `LynxUsbDevice.addConfiguredModule()` and `LynxUsbDevice.getConfiguredModule()` have been replaced with `LynxUsbDevice.getOrAddModule()`.
* Old Blocks for Vuforia and TensorFlow Object Detection are obsolete and have been removed from the
  Blocks editor's toolbox. Existing Blocks OpModes that contain the old Blocks for Vuforia or
  TensorFlow Object Detection can be opened in the Blocks editor, but running them will not work.

### New features
* Adds new `VisionPortal` API for computer vision
    * **This API may be subject to change for final kickoff release!**
    * Several new samples added.
    * Adds support for detecting AprilTags.
    * `VisionPortal` is the new entry point for both AprilTag and TFOD processing.
    * Vuforia will be removed in a future release.
    * Updated TensorFlow dependencies.
    * Added support for webcam camera controls to blocks.
    * The Blocks editor's toolbox now has a Vision category, directly above the Utilities category.
* Related documentation for associated technologies can be found at
    * [AprilTag Introduction](https://ftc-docs.firstinspires.org/apriltag-intro)
    * [AprilTag SDK Guide](https://ftc-docs.firstinspires.org/apriltag-sdk)
    * [AprilTag Detection Values](https://ftc-docs.firstinspires.org/apriltag-detection-values)
    * [AprilTag Test Images](https://ftc-docs.firstinspires.org/apriltag-test-images)
    * [Camera Calibration](https://ftc-docs.firstinspires.org/camera-calibration)
* Adds Driver Station support for Logitech Dual Action and Sony PS5 DualSense gamepads.
    * This **does not** include support for the Sony PS5 DualSense Edge gamepad.
    * Always refer to Game Manual 1 to determine gamepad legality in competition.
* Adds support for MJPEG payload streaming to UVC driver (external JPEG decompression routine required for use).
* Shows a hint on the Driver Station UI about how to bind a gamepad when buttons are pressed or the sticks are moved on an unbound gamepad.
* Adds option for fullscreening "Camera Stream" on Driver Station.
* OnBotJava source code is automatically saved as a ZIP file on every build with a rolling window of the last 30 builds kept; allows recovering source code from previous builds if code is accidentally deleted or corrupted.
* Adds support for changing the addresses of Expansion Hubs that are not connected directly via USB.
  * The Expansion Hub Address Change screen now has an Apply button that changes the addresses without leaving the screen.
  * Addresses that are assigned to other hubs connected to the same USB connection or Control Hub are no longer able to be selected.
* Increases maximum size of Blocks inline comments to 100 characters
* Saves position of open Blocks comment balloons
* Adds new AprilTag Driving samples:  RobotDriveToAprilTagTank & RobotDriveToAprilTagOmni
* Adds Sample to illustrate optimizing camera exposure for AprilTags: ConceptAprilTagOptimizeExposure

### Bug Fixes
* Corrects inspection screen to report app version using the SDK version defined in the libraries instead of the version specified in `AndroidManifest.xml`. This corrects the case where the app could show matching versions numbers to the user but still state that the versions did not match.
  * If the version specified in `AndroidManifest.xml` does not match the SDK version, an SDK version entry will be displayed on the Manage webpage.
* Fixes no error being displayed when saving a configuration file with duplicate names from the Driver Station.
* Fixes a deadlock in the UVC driver which manifested in https://github.com/OpenFTC/EasyOpenCV/issues/57.
* Fixes a deadlock in the UVC driver that could occur when hot-plugging cameras.
* Fixes UVC driver compatibility with Arducam OV9281 global shutter camera.
* Fixes Emergency Stop condition when an OnBotJava build with duplicate OpMode names occurs.
* Fixes known causes of "Attempted use of a closed LynxModule instance" logspam.
* Fixes the visual identification LED pattern when configuring Expansion Hubs connected via RS-485.

## Version 8.1.1 (20221201-150726)

This is a bug fix only release to address the following four issues.

* [Issue #492](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/492) - Can't create new blocks opmodes.
* [Issue #495](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/495) - Remove the final modifier from the OpMode's Telemetry object.
* [Issue #500](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/500) - Some devices cannot be configured when the Driver Station app has been updated to 8.1
  * Updating either the Robot Controller app or the Driver Station app to 8.1.1 or later will fix this issue.
* The Modern Robotics touch sensor was configurable as a  Digital Device. It can only be used as an Analog Device.

## Version 8.1 (20221121-115119)

### Breaking Changes
* Deprecates the `OpMode` fields `msStuckDetectInit`, `msStuckDetectInitLoop`, `msStuckDetectStart`, `msStuckDetectLoop`, and `msStuckDetectStop`.
    * OpModes no longer have a time limit for `init()`, `init_loop()`, `start()` or `loop()`, so the fields corresponding to those methods are no longer used.
    * `stop()` still has a time limit, but it is now hardcoded to be 1 second, and cannot be changed using `msStuckDetectStop`.
* Deprecates the `OpMode` methods `internalPreInit()`, `internalPostInitLoop()`, and `internalPostLoop()`.
    * Iterative `OpMode`s will continue to call these methods in case they were overridden.
    * These methods will not be called at all for `LinearOpMode`s.
* Deprecates (and stops respecting) `DeviceProperties.xmlTagAliases`.

### Enhancements
* Adds a new `IMU` interface to Blocks and Java that can be used with both the original BNO055 IMU
  included in all older Control Hubs and Expansion Hubs, and the new alternative BHI260AP IMU.
  * You can determine which type of IMU is in your Control Hub by navigating to the Manage page of the web interface.
  * To learn how to use the new `IMU` interface, see https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html. The `SensorIMU` Blocks sample was also updated to use the new `IMU` interface, and the following Java samples were added:
    * `SensorIMUOrthogonal`
      * Use this sample if your REV Hub is mounted so that it is parallel or perpendicular to the
        bottom of your robot.
    * `SensorIMUNonOrthogonal`
      * Use this sample if your REV Hub is mounted to your robot in any other orientation
    * `ConceptExploringIMUOrientations`
      * This OpMode is a tool to help you understand how the orthogonal orientations work, and
        which one applies to your robot.
  * The BHI260AP IMU can only be accessed via the new `IMU` interface. The BNO055 IMU can be
    programmed using the new `IMU` interface, or you can continue to program it using the old `BNO055IMU`
    interface. If you want to be able to quickly switch to a new Control Hub that may contain the
    BHI260AP IMU, you should migrate your code to use the new `IMU` interface.
  * Unlike the old `BNO055IMU` interface, which only worked correctly when the REV Hub was mounted flat
    on your robot, the `IMU` interface allows you to specify the orientation of the REV Hub on your
    robot. It will account for this, and give you your orientation in a Robot Coordinate System,
    instead of a special coordinate system for the REV Hub. As a result, your pitch and yaw will be
    0 when your *robot* is level, instead of when the REV Hub is level, which will result in much
    more reliable orientation angle values for most mounting orientations.
  * Because of the new robot-centric coordinate system, the pitch and roll angles returned by the
    `IMU` interface will be different from the ones returned by the `BNO055IMU` interface. When you are
    migrating your code, pay careful attention to the documentation.
  * If you have calibrated your BNO055, you can provide that calibration data to the new `IMU`
    interface by passing a `BNO055IMUNew.Parameters` instance to `IMU.initialize()`.
  * The `IMU` interface is also suitable for implementation by third-party vendors for IMUs that
    support providing the orientation in the form of a quaternion.
* Iterative `OpMode`s (as opposed to `LinearOpMode`s) now run on a dedicated thread.
    * Cycle times should not be as impacted by everything else going on in the system.
    * Slow `OpMode`s can no longer increase the amount of time it takes to process network commands, and vice versa.
    * The `init()`, `init_loop()`, `start()` and `loop()` methods no longer need to return within a certain time frame.
* BNO055 IMU legacy driver: restores the ability to initialize in one OpMode, and then have another OpMode re-use that
  initialization. This allows you to maintain the 0-yaw position between OpModes, if desired.
* Allows customized versions of device drivers in the FTC SDK to use the same XML tag.
  * Before, if you wanted to customize a device driver, you had to copy it to a new class _and_ give
    it a new XML tag. Giving it a new XML tag meant that to switch which driver was being used, you
    had to modify your configuration file.
  * Now, to use your custom driver, all you have to do is specify your custom driver's class when
    calling `hardwareMap.get()`. To go back to the original driver, specify the original driver
    class. If you specify an interface that is implemented by both the original driver and the
    custom driver, there is no guarantee about which implementation will be returned.

### Bug Fixes
* Fixes accessing the "Manage TensorFlow Lite Models" and "Manage Sounds" links and performing
  Blocks and OnBotJava OpMode downloads from the REV Hardware Client.
* Fixes issue where an I2C device driver would be auto-initialized using the parameters assigned in
  a previous OpMode run.
* Improves Driver Station popup menu placement in the landscape layout.
* Fixes NullPointerException when attempting to get a non-configured BNO055 IMU in a Blocks OpMode on an RC phone.
* Fixes problem with Blocks if a variable is named `orientation`.

## Version 8.0 (20220907-131644)

### Breaking Changes
* Increases the Robocol version.
  * This means an 8.0 or later Robot Controller or Driver Station will not be able to communicate with a 7.2 or earlier Driver Station or Robot Controller.
  * If you forget to update both apps at the same time, an error message will be shown explaining which app is older and should be updated.
* Initializing I2C devices now happens when you retrieve them from the `HardwareMap` for the first time.
  * Previously, all I2C devices would be initialized before the OpMode even began executing,
    whether you were actually going to use them or not. This could result in reduced performance and
    unnecessary warnings.
  * With this change, it is very important for Java users to retrieve all needed devices from the
    `HardwareMap` **during the Init phase of the OpMode**. Namely, declare a variable for each hardware
    device the OpMode will use, and assign a value to each. Do not do this during the Run phase, or your
    OpMode may briefly hang while the devices you are retrieving get initialized.
  * OpModes that do not use all of the I2C devices specified in the configuration file should take
    less time to initialize. OpModes that do use all of the specified I2C devices should take the
    same amount of time as previously.
* Fixes [issue #251](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251) by changing the order in which axis rotation rates are read from the angular velocity vector in the BNO055 IMU driver.
* Deprecates `pitchMode` in `BNO055IMU.Parameters`.
  * Setting `pitchMode` to `PitchMode.WINDOWS` would break the coordinate conventions used by the driver.
* Moves `OpModeManagerImpl` to the `com.qualcomm.robotcore.eventloop.opmode` package.
  * This breaks third party libraries EasyOpenCV (version 1.5.1 and earlier) and FTC Dashboard (version 0.4.4 and earlier).
* Deletes the deprecated `OpMode` method `resetStartTime()` (use `resetRuntime()` instead).
* Deletes the protected `LinearOpMode.LinearOpModeHelper` class (which was not meant for use by OpModes).
* Removes I2C Device (Synchronous) config type (deprecated since 2018)

### Enhancements
* Uncaught exceptions in OpModes no longer require a Restart Robot
  * A blue screen popping up with a stacktrace is not an SDK error; this replaces the red text in the telemetry area.
  * Since the very first SDK release, OpMode crashes have put the robot into "EMERGENCY STOP" state, only showing the first line of the exception, and requiring the user to press "Restart Robot" to continue
  * Exceptions during an OpMode now open a popup window with the same color scheme as the log viewer, containing 15 lines of the exception stacktrace to allow easily tracing down the offending line without needing to connect to view logs over ADB or scroll through large amounts of logs in the log viewer.
  * The exception text in the popup window is both zoomable and scrollable just like a webpage.
  * Pressing the "OK" button in the popup window will return to the main screen of the Driver Station and allow an OpMode to be run again immediately, without the need to perform a "Restart Robot"
* Adds new Java sample to demonstrate using a hardware class to abstract robot actuators, and share them across multiple OpModes.
  * Sample OpMode is [ConceptExternalHardwareClass.java](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptExternalHardwareClass.java)
  * Abstracted hardware class is [RobotHardware.java](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/RobotHardware.java)
* Updates RobotAutoDriveByGyro_Linear Java sample to use REV Control/Expansion hub IMU.
* Updates Vuforia samples to reference PowerPlay assets and have correct names and field locations of image targets.
* Updates TensorFlow samples to reference PowerPlay assets.
* Adds opt-in support for Java 8 language features to the OnBotJava editor.
  * To opt in, open the OnBotJava Settings, and check `Enable beta Java 8 support`.
  * Note that Java 8 code will only compile when the Robot Controller runs Android 7.0 Nougat or later.
  * Please report issues [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues).
* In OnBotJava, clicking on build errors now correctly jumps to the correct location.
* Improves OnBotJava autocomplete behavior, to provide better completion options in most cases.
* Adds a QR code to the Robot Controller Inspection Report when viewed from the Driver Station for scanning by inspectors at competition.
* Improves I2C performance and reliability in some scenarios.

## Version 7.2 (20220723-130006)

### Breaking Changes
* Updates the build tooling.  For Android Studio users, this change requires Android Studio Chipmunk 2021.2.1.
* Removes support for devices that are not competition legal, including Modern Robotics Core Control Modules, the Matrix Controller, and HiTechnic/NXT controllers and sensors.  Support remains for Modern Robotics I2C sensors.

### Enhancements
* Increases the height of the 3-dots Landscape menu touch area on the Driver Station, making it much easier to select.
* Adds `terminateOpModeNow()` method to allow OpModes to cleanly self-exit immediately.
* Adds `opModeInInit()` method to `LinearOpMode` to facilitate init-loops. Similar to `opModeIsActive()` but for the init phase.
* Warns user if they have a Logitech F310 gamepad connected that is set to DirectInput mode.
* Allows SPARKmini motor controllers to react more quickly to speed changes.
* Hides the version number of incorrectly installed sister app (i.e. DS installed on RC device or vice-versa) on inspection screen.
* Adds support for allowing the user to edit the comment for the runOpMode block.
* Adds parameterDefaultValues field to @ExportToBlocks. This provides the ability for a java method with an @ExportToBlocks annotation to specify default values for method parameters when it is shown in the block editor.
* Make LinearOpMode blocks more readable. The opmode name is displayed on the runOpMode block, but not on the other LinearOpMode blocks.
* Added support to TensorFlow Object Detection for using a different frame generator, instead of Vuforia.
  Using Vuforia to pass the camera frame to TFOD is still supported.
* Removes usage of Renderscript.
* Fixes logspam on app startup of repeated stacktraces relating to `"Failed resolution of: Landroid/net/wifi/p2p/WifiP2pManager$DeviceInfoListener"`
* Allows disabling bluetooth radio from inspection screen
* Improves warning messages when I2C devices are not responding
* Adds support for controlling the RGB LED present on PS4/Etpark gamepads from OpModes
* Removes legacy Pushbot references from OpMode samples.  Renames "Pushbot" samples to "Robot".  Motor directions reversed to be compatible with "direct Drive" drive train.


### Bug fixes
* Fixes [issue #316](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/316) (MatrixF.inverted() returned an incorrectly-sized matrix for 1x1 and 2x2 matrixes).
* Self inspect now allows for Driver Station and Robot Controller compatibility between point releases.
* Fixes bug where if the same `RumbleEffect` object instance was queued for multiple gamepads, it
  could happen that both rumble commands would be sent to just one gamepad.
* Fixes bug in Driver Station where on the Driver Hub, if Advanced Gamepad Features was disabled and
  an officially supported gamepad was connected, then opening the Advanced Gamepad Features or
  Gamepad Type Overrides screens would cause the gamepad to be rebound by the custom USB driver even
  though advanced gamepad features was disabled.
* Protects against (unlikely) null pointer exception in Vuforia Localizer.
* Harden OnBotJava and Blocks saves to protect against save issues when disconnecting from Program and Manage
* Fixes issue where the RC app would hang if a REV Hub I2C write failed because the previous I2C
  operation was still in progress. This hang most commonly occurred during REV 2M Distance Sensor initialization
* Removes ConceptWebcam.java sample program.  This sample is not compatible with OnBotJava.
* Fixes bug where using html tags in an @ExportToBlocks comment field prevented the blocks editor from loading.
* Fixes blocks editor so it doesn't ask you to save when you haven't modified anything.
* Fixes uploading a very large blocks project to offline blocks editor.
* Fixes bug that caused blocks for DcMotorEx to be omitted from the blocks editor toolbox.
* Fixes [Blocks Programs Stripped of Blocks (due to using TensorFlow Label block)](https://ftcforum.firstinspires.org/forum/ftc-technology/blocks-programming/87035-blocks-programs-stripped-of-blocks)

## Version 7.1 (20211223-120805)

* Fixes crash when calling `isPwmEnabled()` ([issue #223](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/233)).
* Fixes lint error ([issue #4](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/4)).
* Fixes Driver Station crash when attempting to use DualShock4 v1 gamepad with Advanced Gamepad Features enabled ([issue #173](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/173)).
* Fixes possible (but unlikely) Driver Station crash when connecting gamepads of any type.
* Fixes bug where Driver Station would use generic 20% deadzone for Xbox360 and Logitech F310 gamepads when Advanced Gamepad Features was disabled.
* Added SimpleOmniDrive sample OpMode.
* Adds UVC white balance control API.
* Fixes [issue #259](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/259) Most blocks samples for TensorFlow can't be used for a different model.
    * The blocks previously labeled TensorFlowObjectDetectionFreightFrenzy (from the subcategory named "Optimized for Freight Frenzy") and TensorFlowObjectDetectionCustomModel (from the subcategory named "Custom Model") have been replaced with blocks labeled TensorFlowObjectDetection. Blocks in existing opmodes will be automatically updated to the new blocks when opened in the blocks editor.
* Fixes [issue #260](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/260) Blocks can't call java method that has a VuforiaLocalizer parameter.
    * Blocks now has a block labeled VuforiaFreightFrenzy.getVuforiaLocalizer for this.
* Added a page to manage the TensorFlow Lite models in /sdcard/FIRST/tflitemodels. To get to the TFLite Models page:
    * You can click on the link at the bottom of the the Manage page.
    * You can click on the link at the upper-right the Blocks project page.
* Fixes logspam when `isBusy()` is called on a motor not in RTP mode.
* Hides the "RC Password" item on the inspection screen for phone-based Robot Controllers. (It is only applicable for Control Hubs).
* Adds channel 165 to Wi-Fi Direct channel selection menu in the settings screen. (165 was previously available through the web UI, but not locally in the app).

## Version 7.0 (20210915-141025)

### Enhancements and New Features
* Adds support for external libraries to OnBotJava and Blocks.
    * Upload .jar and .aar files in OnBotJava.
      * Known limitation - RobotController device must be running Android 7.0 or greater.
      * Known limitation - .aar files with assets are not supported.
    * External libraries can provide support for hardware devices by using the annotation in the
      com.qualcomm.robotcore.hardware.configuration.annotations package.
    * External libraries can include .so files for native code.
    * External libraries can be used from OnBotJava OpModes.
    * External libraries that use the following annotations can be used from Blocks OpModes.
      * org.firstinspires.ftc.robotcore.external.ExportClassToBlocks
      * org.firstinspires.ftc.robotcore.external.ExportToBlocks
    * External libraries that use the following annotations can add new hardware devices:
      * com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType
      * com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
      * com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType
      * com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
      * com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
      * com.qualcomm.robotcore.hardware.configuration.annotations.ServoType
    * External libraries that use the following annotations can add new functionality to the Robot Controller:
      * org.firstinspires.ftc.ftccommon.external.OnCreate
      * org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
      * org.firstinspires.ftc.ftccommon.external.OnCreateMenu
      * org.firstinspires.ftc.ftccommon.external.OnDestroy
      * org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
* Adds support for REV Robotics Driver Hub.
* Adds fully custom userspace USB gamepad driver to Driver Station (see "Advanced Gamepad Features" menu in DS settings).
    * Allows gamepads to work on devices without native Linux kernel support (e.g. some Romanian Motorola devices).
    * Allows the DS to read the unique serial number of each gamepad, enabling auto-recovery of dropped gamepads even if two gamepads of the same model drop. *(NOTE: unfortunately this does not apply to Etpark gamepads, because they do not have a unique serial)*.
    * Reading the unique serial number also provides the ability to configure the DS to assign gamepads to a certain position by default (so no need to do start+a/b at all).
    * The LED ring on the Xbox360 gamepad and the RGB LED bar on the PS4 gamepad is used to indicate the driver position the gamepad is bound to.
    * The rumble motors on the Xbox360, PS4, and Etpark gamepads can be controlled from OpModes.
    * The 2-point touchpad on the PS4 gamepad can be read from OpModes.
    * The "back" and "guide" buttons on the gamepad can now be safely bound to robot controls (Previously, on many devices, Android would intercept these buttons as home button presses and close the app).
    * Advanced Gamepad features are enabled by default, but may be disabled through the settings menu in order to revert to gamepad support provided natively by Android.
* Improves accuracy of ping measurement.
    * Fixes issue where the ping time showed as being higher than reality when initially connecting to or restarting the robot.
    * To see the full improvement, you must update both the Robot Controller and Driver Station apps.
* Updates samples located at [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples).
    * Added ConceptGamepadRumble and ConceptGamepadTouchpad samples to illustrate the use of these new gampad capabilities.
    * Condensed existing Vuforia samples into just 2 samples (ConceptVuforiaFieldNavigation & ConceptVuforiaFieldNavigationWebcam) showing how to determine the robot's location on the field using Vuforia. These both use the current season's Target images.
    * Added ConceptVuforiaDriveToTargetWebcam to illustrate an easy way to drive directly to any visible Vuforia target.
* Makes many improvements to the warning system and individual warnings.
    * Warnings are now much more spaced out, so that they are easier to read.
    * New warnings were added for conditions that should be resolved before competing.
    * The mismatched apps warning now uses the major and minor app versions, not the version code.
    * The warnings are automatically re-enabled when a Robot Controller app from a new FTC season is installed.
* Adds support for I2C transactions on the Expansion Hub / Control Hub without specifying a register address.
    * See section 3 of the [TI I2C spec](https://www.ti.com/lit/an/slva704/slva704.pdf).
    * Calling these new methods when using Modern Robotics hardware will result in an UnsupportedOperationException.
* Changes VuforiaLocalizer `close()` method to be public.
* Adds support for TensorFlow v2 object detection models.
* Reduces ambiguity of the Self Inspect language and graphics.
* OnBotJava now warns about potentially unintended file overwrites.
* Improves behavior of the Wi-Fi band and channel selector on the Manage webpage.

### Bug fixes
 * Fixes Robot Controller app crash on Android 9+ when a Driver Station connects.
 * Fixes issue where an OpMode was responsible for calling shutdown on the
   TensorFlow TFObjectDetector. Now this is done automatically.
 * Fixes Vuforia initialization blocks to allow user to chose AxesOrder. Updated
   relevant blocks sample opmodes.
 * Fixes [FtcRobotController issue #114](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/114)
   LED blocks and Java class do not work.
 * Fixes match logging for OpModes that contain special characters in their names.
 * Fixes Driver Station OpMode controls becoming unresponsive if the Driver Station was set to the landscape layout and an OnBotJava build was triggered while an OpMode was running.
 * Fixes the Driver Station app closing itself when it is switched away from, or the screen is turned off.
 * Fixes "black swirl of doom" (Infinite "configuring Wi-Fi Direct" message) on older devices.
 * Updates the wiki comment on the OnBotJava intro page.

## Version 6.2 (20210218-074821)

### Enhancements
* Attempts to automatically fix the condition where a Control Hub's internal Expansion Hub is not
  working by re-flashing its firmware
* Makes various improvements to the Wi-Fi Direct pairing screen, especially in landscape mode
* Makes the Robot Controller service no longer be categorically restarted when the main activity is brought to foreground
    * (e.g. the service is no longer restarted simply by viewing the Self Inspect screen and pressing the back button)
    * It is still restarted if the Settings menu or Configure Robot menu is opened


### Bug fixes
* Fixes [FtcRobotController issue #71](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/71)
  Cannot open OpModes in v6.1 Blocks offline editor
* Fixes [FtcRobotController issue #79](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/79)
  6.1 causes a soft reboot on the Motorola E5 Play
* Fixes issue where the Control Hub OS's watchdog would restart the Robot Controller app if
  the Control Hub was not able to communicate with its internal Expansion Hub
* Fixes certain I2C devices not showing up in the appropriate `HardwareMap` fields (such as `hardwareMap.colorSensor`)
* Fixes issue where performing a Wi-Fi factory reset on the Control Hub would not set the Wi-Fi band to 2.4 GHz
* Fixes issue where OnBotJava might fail to create a new file if the option to "Setup Code for Configured Hardware" was selected
* Fixes issue where performing certain operations after an OpMode crashes would temporarily break Control/Expansion Hub communication
* Fixes issue where a Control Hub with a configured USB-connected Expansion Hub would not work if the Expansion Hub was missing at startup
* Fixes potential issues caused by having mismatched Control/Expansion Hub firmware versions
* Fixes [ftc_app issue 673](https://github.com/ftctechnh/ftc_app/issues/673) Latest matchlog is being deleted instead of old ones by RobotLog
* Fixes ConceptVuforiaUltimateGoalNavigationWebcam sample opmode by correctly orienting camera on robot.
* Fixes issue where logcat would be spammed with InterruptedExceptions when stop is requested from the Driver Station (this behavior was accidentally introduced in v5.3). This change has no impact on functionality.
* Fixes issue where the blocks editor fails to load if the name of any TeleOp opmode contains an apostrophe.

## Version 6.1 (20201209-113742)
* Makes the scan button on the configuration screen update the list of Expansion Hubs connected via RS-485
    * Fixes [SkyStone issue #143](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/143)
* Improves web interface compatibility with older browser and Android System WebView versions.
* Fixes issue in UVC driver where some cameras (e.g. certain MS Lifecams) which reported frame intervals as rounded rather than truncated values (e.g. `666667*100ns` instead of `666666*100ns` for 15FPS) would fail to start streaming.
* Adds support in UVC driver for virtual PTZ control
* Adds support in UVC driver for gain (ISO) control
* Adds support in UVC driver for enabling/disable AE priority. This setting provides a means to tell the camera firmware either
    * A) It can undershoot the requested frame rate in order to provide a theoretically better image (i.e. with a longer exposure than the inter-frame period of the selected frame rate allows)
    * B) It *must* meet the inter-frame deadline for the selected frame rate, even if the image may be underexposed as a result
* Adds support for the Control Hub OS 1.1.2 Robot Controller watchdog
    * The Robot Controller app will be restarted if it stops responding for more than 10 seconds
* Adds support for using the Driver Station app on Android 10+
* Introduces an automatic TeleOp preselection feature
    * For details and usage guide, please see [this wiki entry](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Automatically-Loading-a-Driver-Controlled-Op-Mode)
* Shows icon next to OpMode name in the OpMode list dropdown on the Driver Station to indicate the source of the OpMode (i.e. the programming tool used to create it)
* Fixes issue where the Driver Station app would exit after displaying the Configuring Wi-Fi Direct screen
* Fixes Blocks and OnBotJava prompts when accessed via the REV Hardware Client

## Version 6.0 (20200921-085816)

### Important Notes
* Version 6.0 is the version for the Ultimate Goal season.
* Requires Android Studio 4.0.
* Android Studio users need to be connected to the Internet the first time they build the app (in order to download needed packages for the build).
* Version 5.5 was a moderately large off-season, August 2020, drop.  It's worth reviewing those release notes below also.
* Version 5.5 and greater will not work on older Android 4.x and 5.x phones.  Users must upgrade to an approved Android 6.x device or newer.
* The default PIDF values for REV motors have been reverted to the default PID values that were used in the 2018-2019 season
    * This change was made because the 2018-2019 values turned out to work better for many mechanisms
    * This brings the behavior of the REV motors in line with the behavior of all other motors
    * If you prefer the 2019-2020 season's behavior for REV motors, here are the PIDF values that were in place, so that you can manually set them in your OpModes:
      <br>
      **HD Hex motors (all gearboxes):**
      Velocity PIDF values: `P = 1.17`, `I = 0.117`, `F = 11.7`
      Position PIDF values: `P = 5.0`
      **Core Hex motor:**
      Velocity PIDF values: `P = 4.96`, `I = 0.496`, `F = 49.6`
      Position PIDF values: `P = 5.0`

### New features
* Includes TensorFlow inference model and sample OpModes to detect Ultimate Goal Starter Stacks (four rings vs single ring stack).
* Includes Vuforia Ultimate Goal vision targets and sample OpModes.
* Introduces a digital zoom feature for TensorFlow object detection (to detect objects more accurately at greater distances).
* Adds configuration entry for the REV UltraPlanetary HD Hex motor

### Enhancements
* Adds setGain() and getGain() methods to the NormalizedColorSensor interface
    * By setting the gain of a color sensor, you can adjust for different lighting conditions.
      For example, if you detect lower color values than expected, you can increase the gain.
    * The gain value is only applied to the argb() and getNormalizedColors() methods, not to the raw color methods.
      The getNormalizedColors() method is recommended for ease-of-use and clarity, since argb() has to be converted.
    * Updates SensorColor Java sample to demonstrate gain usage
* Merges SensorREVColorDistance Java sample into SensorColor Java sample, which showcases best practices for all color sensors
* Improves retrieving values from the REV Color Sensor V3
    * Updates the normalization calculation of the RGB channels
    * Improves the calculation of the alpha channel (can be used as an overall brightness indicator)
    * Fixes the default sensor resolution, which caused issues with bright environments
    * Adds support for changing the resolution and measuring rate of the Broadcom sensor chip
    * Removes IR readings and calculations not meant for the Broadcom sensor chip

### Bug fixes
* Improves reliability of BNO055IMU IMU initialization to prevent random initialization failures (which manifested as `Problem with 'imu'`).

## Version 5.5 (20200824-090813)

Version 5.5 requires Android Studio 4.0 or later.

### New features
* Adds support for calling custom Java classes from Blocks OpModes (fixes [SkyStone issue #161](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/161)).
    * Classes must be in the org.firstinspires.ftc.teamcode package.
    * To have easy access to the opMode, hardwareMap, telemetry, gamepad1, and gamepad2, classes can
      extends org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.
    * Methods must be public static and have no more than 21 parameters.
    * Methods must be annotated with org.firstinspires.ftc.robotcore.external.ExportToBlocks.
    * Parameters declared as OpMode, LinearOpMode, Telemetry, and HardwareMap are supported and the
      argument is provided automatically, regardless of the order of the parameters. On the block,
      the sockets for those parameters are automatically filled in.
    * Parameters declared as char or java.lang.Character will accept any block that returns text
      and will only use the first character in the text.
    * Parameters declared as boolean or java.lang.Boolean will accept any block that returns boolean.
    * Parameters declared as byte, java.lang.Byte, short, java.lang.Short, int, java.lang.Integer,
      long, or java.lang.Long,  will accept any block that returns a number and will round that
      value to the nearest whole number.
    * Parameters declared as float, java.lang.Float, double, java.lang.Double will accept any
      block that returns a number.
* Adds telemetry API method for setting display format
    * Classic
    * Monospace
    * HTML (certain tags only)
* Adds blocks support for switching cameras.
* Adds Blocks support for TensorFlow Object Detection with a custom model.
* Adds support for uploading a custom TensorFlow Object Detection model in the Manage page, which
  is especially useful for Blocks and OnBotJava users.
* Shows new Control Hub blink codes when the Wi-Fi band is switched using the Control Hub's button (only possible on Control Hub OS 1.1.2)
* Adds new warnings which can be disabled in the Advanced RC Settings
    * Mismatched app versions warning
    * Unnecessary 2.4 GHz Wi-Fi usage warning
    * REV Hub is running outdated firmware (older than version 1.8.2)
* Adds support for Sony PS4 gamepad, and reworks how gamepads work on the Driver Station
    * Removes preference which sets gamepad type based on driver position. Replaced with menu which allows specifying type for gamepads with unknown VID and PID
	* Attempts to auto-detect gamepad type based on USB VID and PID
	* If gamepad VID and PID is not known, use type specified by user for that VID and PID
	* If gamepad VID and PID is not known AND the user has not specified a type for that VID and PID, an educated guess is made about how to map the gamepad
* Driver Station will now attempt to automatically recover from a gamepad disconnecting, and re-assign it to the position it was assigned to when it dropped
    * If only one gamepad is assigned and it drops: it can be recovered
    * If two gamepads are assigned, and have **different** VID/PID signatures, and only one drops: it will be recovered
    * If two gamepads are assigned, and have **different** VID/PID signatures, and BOTH drop: both will be recovered
    * If two gamepads are assigned, and have **the same** VID/PID signatures, and only one drops: it will be recovered
    * If two gamepads are assigned, and have **the same** VID/PID signatures, and BOTH drop: **neither** will be recovered, because of the ambiguity of the gamepads when they re-appear on the USB bus.
    * There is currently one known edge case: if there are **two** gamepads with **the same** VID/PID signature plugged in, **but only one is assigned**, and they BOTH drop, it's a 50-50 chance of which one will be chosen for automatic recovery to the assigned position: it is determined by whichever one is re-enumerated first by the USB bus controller.
* Adds landscape user interface to Driver Station
    * New feature: practice timer with audio cues
    * New feature (Control Hub only): wireless network connection strength indicator (0-5 bars)
    * New feature (Control Hub only): tapping on the ping/channel display will switch to an alternate display showing radio RX dBm and link speed (tap again to switch back)
    * The layout will NOT autorotate. You can switch the layout from the Driver Station's settings menu.
### Breaking changes
* Removes support for Android versions 4.4 through 5.1 (KitKat and Lollipop). The minSdkVersion is now 23.
* Removes the deprecated `LinearOpMode` methods `waitOneFullHardwareCycle()` and `waitForNextHardwareCycle()`
### Enhancements
* Handles RS485 address of Control Hub automatically
    * The Control Hub is automatically given a reserved address
    * Existing configuration files will continue to work
    * All addresses in the range of 1-10 are still available for Expansion Hubs
    * The Control Hub light will now normally be solid green, without blinking to indicate the address
    * The Control Hub will not be shown on the Expansion Hub Address Change settings page
* Improves REV Hub firmware updater
    * The user can now choose between all available firmware update files
    * Version 1.8.2 of the REV Hub firmware is bundled into the Robot Controller app.
    * Text was added to clarify that Expansion Hubs can only be updated via USB.
    * Firmware update speed was reduced to improve reliability
    * Allows REV Hub firmware to be updated directly from the Manage webpage
* Improves log viewer on Robot Controller
    * Horizontal scrolling support (no longer word wrapped)
    * Supports pinch-to-zoom
    * Uses a monospaced font
    * Error messages are highlighted
    * New color scheme
* Attempts to force-stop a runaway/stuck OpMode without restarting the entire app
    * Not all types of runaway conditions are stoppable, but if the user code attempts to talk to hardware during the runaway, the system should be able to capture it.
* Makes various tweaks to the Self Inspect screen
    * Renames "OS version" entry to "Android version"
    * Renames "Wi-Fi Direct Name" to "Wi-Fi Name"
    * Adds Control Hub OS version, when viewing the report of a Control Hub
    * Hides the airplane mode entry, when viewing the report of a Control Hub
    * Removes check for ZTE Speed Channel Changer
    * Shows firmware version for **all** Expansion and Control Hubs
* Reworks network settings portion of Manage page
    * All network settings are now applied with a single click
    * The Wi-Fi Direct channel of phone-based Robot Controllers can now be changed from the Manage page
    * Wi-Fi channels are filtered by band (2.4 vs 5 GHz) and whether they overlap with other channels
    * The current Wi-Fi channel is pre-selected on phone-based Robot Controllers, and Control Hubs running OS 1.1.2 or later.
    * On Control Hubs running OS 1.1.2 or later, you can choose to have the system automatically select a channel on the 5 GHz band
* Improves OnBotJava
    * New light and dark themes replace the old themes (chaos, github, chrome,...)
        * the new default theme is `light` and will be used when you first update to this version
    * OnBotJava now has a tabbed editor
    * Read-only offline mode
* Improves function of "exit" menu item on Robot Controller and Driver Station
    * Now guaranteed to be fully stopped and unloaded from memory
* Shows a warning message if a LinearOpMode exists prematurely due to failure to monitor for the start condition
* Improves error message shown when the Driver Station and Robot Controller are incompatible with each other
* Driver Station OpMode Control Panel now disabled while a Restart Robot is in progress
* Disables advanced settings related to Wi-Fi Direct when the Robot Controller is a Control Hub.
* Tint phone battery icons on Driver Station when low/critical.
* Uses names "Control Hub Portal" and "Control Hub" (when appropriate) in new configuration files
* Improve I2C read performance
    * Very large improvement on Control Hub; up to ~2x faster with small (e.g. 6 byte) reads
    * Not as apparent on Expansion Hubs connected to a phone
* Update/refresh build infrastructure
    * Update to 'androidx' support library from 'com.android.support:appcompat', which is end-of-life
    * Update targetSdkVersion and compileSdkVersion to 28
    * Update Android Studio's Android plugin to latest
    * Fix reported build timestamp in 'About' screen
* Add sample illustrating manual webcam use: ConceptWebcam


### Bug fixes
* Fixes [SkyStone issue #248](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/248)
* Fixes [SkyStone issue #232](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/232) and
  modifies bulk caching semantics to allow for cache-preserving MANUAL/AUTO transitions.
* Improves performance when REV 2M distance sensor is unplugged
* Improves readability of Toast messages on certain devices
* Allows a Driver Station to connect to a Robot Controller after another has disconnected
* Improves generation of fake serial numbers for UVC cameras which do not provide a real serial number
    * Previously some devices would assign such cameras a serial of `0:0` and fail to open and start streaming
	* Fixes [ftc_app issue #638](https://github.com/ftctechnh/ftc_app/issues/638).
* Fixes a slew of bugs with the Vuforia camera monitor including:
    * Fixes bug where preview could be displayed with a wonky aspect ratio
    * Fixes bug where preview could be cut off in landscape
    * Fixes bug where preview got totally messed up when rotating phone
    * Fixes bug where crosshair could drift off target when using webcams
* Fixes issue in UVC driver on some devices ([ftc_app 681](https://github.com/ftctechnh/ftc_app/issues/681)) if streaming was started/stopped multiple times in a row
    * Issue manifested as kernel panic on devices which do not have [this kernel patch](https://lore.kernel.org/patchwork/patch/352400/).
    * On affected devices which **do** have the patch, the issue was manifest as simply a failure to start streaming.
    * The Tech Team believes that the root cause of the issue is a bug in the Linux kernel XHCI driver. A workaround was implemented in the SDK UVC driver.
* Fixes bug in UVC driver where often half the frames from the camera would be dropped (e.g. only 15FPS delivered during a streaming session configured for 30FPS).
* Fixes issue where TensorFlow Object Detection would show results whose confidence was lower than
  the minimum confidence parameter.
* Fixes a potential exploitation issue of [CVE-2019-11358](https://www.cvedetails.com/cve/CVE-2019-11358/) in OnBotJava
* Fixes changing the address of an Expansion Hub with additional Expansion Hubs connected to it
* Preserves the Control Hub's network connection when "Restart Robot" is selected
* Fixes issue where device scans would fail while the Robot was restarting
* Fix RenderScript usage
    * Use androidx.renderscript variant: increased compatibility
    * Use RenderScript in Java mode, not native: simplifies build
* Fixes webcam-frame-to-bitmap conversion problem: alpha channel wasn't being initialized, only R, G, & B
* Fixes possible arithmetic overflow in Deadline
* Fixes deadlock in Vuforia webcam support which could cause 5-second delays when stopping OpMode

## Version 5.4 (20200108-101156)
* Fixes [SkyStone issue #88](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/88)
* Adds an inspection item that notes when a robot controller (Control Hub) is using the factory default password.
* Fixes [SkyStone issue #61](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/61)
* Fixes [SkyStone issue #142](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/142)
* Fixes [ftc_app issue #417](https://github.com/ftctechnh/ftc_app/issues/417) by adding more current and voltage monitoring capabilities for REV Hubs.
* Fixes [a crash sometimes caused by OnBotJava activity](https://ftcforum.firstinspires.org/forum/ftc-technology/76217-onbotjava-crashes-robot-controller)
* Improves OnBotJava autosave functionality [ftc_app #738](https://github.com/ftctechnh/ftc_app/issues/738)
* Fixes system responsiveness issue when an Expansion Hub is disconnected
* Fixes issue where IMU initialization could prevent OpModes from stopping
* Fixes issue where AndroidTextToSpeech.speak() would fail if it was called too early
* Adds telemetry.speak() methods and blocks, which cause the Driver Station (if also updated) to speak text
* Adds and improves Expansion Hub-related warnings
    * Improves Expansion Hub low battery warning
        * Displays the warning immediately after the hub reports it
        * Specifies whether the condition is current or occurred temporarily during an OpMode run
        * Displays which hubs reported low battery
    * Displays warning when hub loses and regains power during an OpMode run
        * Fixes the hub's LED pattern after this condition
    * Displays warning when Expansion Hub is not responding to commands
        * Specifies whether the condition is current or occurred temporarily during an OpMode run
    * Clarifies warning when Expansion Hub is not present at startup
        * Specifies that this condition requires a Robot Restart before the hub can be used.
        * The hub light will now accurately reflect this state
    * Improves logging and reduces log spam during these conditions
* Syncs the Control Hub time and timezone to a connected web browser programming the robot, if a Driver Station is not available.
* Adds bulk read functionality for REV Hubs
  * A bulk caching mode must be set at the Hub level with `LynxModule#setBulkCachingMode()`. This applies to all relevant SDK hardware classes that reference that Hub.
  * The following following Hub bulk caching modes are available:
    * `BulkCachingMode.OFF` (default): All hardware calls operate as usual. Bulk data can read through `LynxModule#getBulkData()` and processed manually.
    * `BulkCachingMode.AUTO`: Applicable hardware calls are served from a bulk read cache that is cleared/refreshed automatically to ensure identical commands don't hit the same cache. The cache can also be cleared manually with `LynxModule#clearBulkCache()`, although this is not recommended.
    * (advanced users) `BulkCachingMode.MANUAL`: Same as `BulkCachingMode.AUTO` except the cache is never cleared automatically. To avoid getting stale data, the cache must be manually cleared at the beginning of each loop body or as the user deems appropriate.
* Removes PIDF Annotation values added in Rev 5.3 (to AndyMark, goBILDA and TETRIX motor configurations).
  * The new motor types will still be available but their Default control behavior will revert back to Rev 5.2
* Adds new `ConceptMotorBulkRead` sample Opmode to demonstrate and compare Motor Bulk-Read modes for reducing I/O latencies.

## Version 5.3 (20191004-112306)
* Fixes external USB/UVC webcam support
* Makes various bugfixes and improvements to Blocks page, including but not limited to:
    * Many visual tweaks
    * Browser zoom and window resize behave better
    * Resizing the Java preview pane works better and more consistently across browsers
    * The Java preview pane consistently gets scrollbars when needed
    * The Java preview pane is hidden by default on phones
    * Internet Explorer 11 should work
    * Large dropdown lists display properly on lower res screens
    * Disabled buttons are now visually identifiable as disabled
    * A warning is shown if a user selects a TFOD sample, but their device is not compatible
    * Warning messages in a Blocks OpMode are now visible by default.
* Adds goBILDA 5201 and 5202 motors to Robot Configurator
* Adds PIDF Annotation values to AndyMark, goBILDA and TETRIX motor configurations.
    This has the effect of causing the RUN_USING_ENCODERS and RUN_TO_POSITION modes to use
    PIDF vs PID closed loop control on these motors.  This should provide more responsive, yet stable, speed control.
    PIDF adds Feedforward control to the basic PID control loop.
    Feedforward is useful when controlling a motor's speed because it "anticipates" how much the control voltage
    must change to achieve a new speed set-point, rather than requiring the integrated error to change sufficiently.
    The PIDF values were chosen to provide responsive, yet stable, speed control on a lightly loaded motor.
    The more heavily a motor is loaded (drag or friction), the more noticable the PIDF improvement will be.
* Fixes startup crash on Android 10
* Fixes [ftc_app issue #712](https://github.com/ftctechnh/ftc_app/issues/712) (thanks to FROGbots-4634)
* Fixes [ftc_app issue #542](https://github.com/ftctechnh/ftc_app/issues/542)
* Allows "A" and lowercase letters when naming device through RC and DS apps.

## Version 5.2 (20190905-083277)
* Fixes extra-wide margins on settings activities, and placement of the new configuration button
* Adds Skystone Vuforia image target data.
   * Includes sample Skystone Vuforia Navigation OpModes (Java).
   * Includes sample Skystone Vuforia Navigation OpModes (Blocks).
* Adds TensorFlow inference model (.tflite) for Skystone game elements.
   * Includes sample Skystone TensorFlow OpModes (Java).
   * Includes sample Skystone TensorFlow OpModes (Blocks).
* Removes older (season-specific) sample OpModes.
* Includes 64-bit support (to comply with [Google Play requirements](https://android-developers.googleblog.com/2019/01/get-your-apps-ready-for-64-bit.html)).
* Protects against Stuck OpModes when a Restart Robot is requested. (Thanks to FROGbots-4634) ([ftc_app issue #709](https://github.com/ftctechnh/ftc_app/issues/709))
* Blocks related changes:
   * Fixes bug with blocks generated code when hardware device name is a java or javascript reserved word.
   * Shows generated java code for blocks, even when hardware items are missing from the active configuration.
   * Displays warning icon when outdated Vuforia and TensorFlow blocks are used ([SkyStone issue #27](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/27))

## Version 5.1 (20190820-222104)
* Defines default PIDF parameters for the following motors:
    * REV Core Hex Motor
    * REV 20:1 HD Hex Motor
    * REV 40:1 HD Hex Motor
* Adds back button when running on a device without a system back button (such as a Control Hub)
* Allows a REV Control Hub to update the firmware on a REV Expansion Hub via USB
* Fixes [SkyStone issue #9](https://github.com/FIRST-Tech-Challenge/SkyStone/issues/9)
* Fixes [ftc_app issue #715](https://github.com/ftctechnh/ftc_app/issues/715)
* Prevents extra DS User clicks by filtering based on current state.
* Prevents incorrect DS UI state changes when receiving new OpMode list from RC
* Adds support for REV Color Sensor V3
* Adds a manual-refresh DS Camera Stream for remotely viewing RC camera frames.
    * To show the stream on the DS, initialize **but do not run** a stream-enabled opmode, select the Camera Stream option in the DS menu, and tap the image to refresh. This feature is automatically enabled when using Vuforia or TFOD—no additional RC configuration is required for typical use cases. To hide the stream, select the same menu item again.
    * Note that gamepads are disabled and the selected opmode cannot be started while the stream is open as a safety precaution.
    * To use custom streams, consult the API docs for `CameraStreamServer#setSource` and `CameraStreamSource`.
* Adds many Star Wars sounds to RobotController resources.
* Added Skystone Sounds Chooser Sample Program.
* Switches out startup, connect chimes, and error/warning sounds for Star Wars sounds
* Updates OnBot Java to use a WebSocket for communication with the robot
    * The OnBot Java page no longer has to do a full refresh when a user switches from editing one file to another

Known issues:
* Camera Stream
    * The Vuforia camera stream inherits the issues present in the phone preview (namely [ftc_app issue #574](https://github.com/ftctechnh/ftc_app/issues/574)). This problem does not affect the TFOD camera stream even though it receives frames from Vuforia.
    * The orientation of the stream frames may not always match the phone preview. For now, these frames may be rotated manually via a custom `CameraStreamSource` if desired.
* OnBotJava
    * Browser back button may not always work correctly
    * It's possible for a build to be queued, but not started. The OnBot Java build console will display a warning if this occurs.
    * A user might not realize they are editing a different file if the user inadvertently switches from one file to another since this switch is now seamless. The name of the currently open file is displayed in the browser tab.

## Version 5.0 (built on 19.06.14)
 * Support for the REV Robotics Control Hub.
 * Adds a Java preview pane to the Blocks editor.
 * Adds a new offline export feature to the Blocks editor.
 * Display Wi-Fi channel in Network circle on Driver Station.
 * Adds calibration for Logitech C270
 * Updates build tooling and target SDK.
 * Compliance with Google's permissions infrastructure (Required after build tooling update).
 * Keep Alives to mitigate the Motorola Wi-Fi scanning problem.  Telemetry substitute no longer necessary.
 * Improves Vuforia error reporting.
 * Fixes ftctechnh/ftc_app issues 621, 713.
 * Miscellaneous bug fixes and improvements.

## Version 4.3 (built on 18.10.31)
 * Includes missing TensorFlow-related libraries and files.

## Version 4.2 (built on 18.10.30)
 * Includes fix to avoid deadlock situation with WatchdogMonitor which could result in USB communication errors.
     - Comm error appeared to require that user disconnect USB cable and restart the Robot Controller app to recover.
     - robotControllerLog.txt would have error messages that included the words "E RobotCore: lynx xmit lock: #### abandoning lock:"
 * Includes fix to correctly list the parent module address for a REV Robotics Expansion Hub in a configuration (.xml) file.
     - Bug in versions 4.0 and 4.1 would incorrect list the address module for a parent REV Robotics device as "1".
     - If the parent module had a higher address value than the daisy-chained module, then this bug would prevent the Robot Controller from communicating with the downstream Expansion Hub.
 * Added requirement for ACCESS_COARSE_LOCATION to allow a Driver Station running Android Oreo to scan for Wi-Fi Direct devices.
 * Added google() repo to build.gradle because aapt2 must be downloaded from the google() repository beginning with version 3.2 of the Android Gradle Plugin.
     - Important Note: Android Studio users will need to be connected to the Internet the first time build the ftc_app project.
     - Internet connectivity is required for the first build so the appropriate files can be downloaded from the Google repository.
     - Users should not need to be connected to the Internet for subsequent builds.
     - This should also fix buid issue where Android Studio would complain that it "Could not find com.android.tools.lint:lint-gradle:26.1.4" (or similar).
 * Added support for REV Spark Mini motor controller as part of the configuration menu for a servo/PWM port on the REV Expansion Hub.
 * Provide examples for playing audio files in an OpMode.
 * Block Development Tool Changes
     - Includes a fix for a problem with the Velocity blocks that were reported in the FTC Technology forum (Blocks Programming subforum).
     - Change the "Save completed successfully." message to a white color so it will contrast with a green background.
     - Fixed the "Download image" feature so it will work if there are text blocks in the OpMode.
 * Introduce support for Google's TensorFlow Lite technology for object detetion for 2018-2019 game.
     - TensorFlow lite can recognize Gold Mineral and Silver Mineral from 2018-2019 game.
     - Example Java and Block OpModes are included to show how to determine the relative position of the gold block (left, center, right).

## Version 4.1 (released on 18.09.24)

Changes include:
 * Fix to prevent crash when deprecated configuration annotations are used.
 * Change to allow FTC Robot Controller APK to be auto-updated using FIRST Global Control Hub update scripts.
 * Removed samples for non supported / non legal hardware.
 * Improvements to Telemetry.addData block with "text" socket.
 * Updated Blocks sample OpMode list to include Rover Ruckus Vuforia example.
 * Update SDK library version number.

## Version 4.0 (released on 18.09.12)

Changes include:
 * Initial support for UVC compatible cameras
    - If UVC camera has a unique serial number, RC will detect and enumerate by serial number.
    - If UVC camera lacks a unique serial number, RC will only support one camera of that type connected.
    - Calibration settings for a few cameras are included (see TeamCode/src/main/res/xml/teamwebcamcalibrations.xml for details).
    - User can upload calibration files from Program and Manage web interface.
    - UVC cameras seem to draw a fair amount of electrical current from the USB bus.
         + This does not appear to present any problems for the REV Robotics Control Hub.
	 + This does seem to create stability problems when using some cameras with an Android phone-based Robot Controller.
	 + FTC Tech Team is investigating options to mitigate this issue with the phone-based Robot Controllers.
    - Updated sample Vuforia Navigation and VuMark OpModes to demonstrate how to use an internal phone-based camera and an external UVC webcam.

 * Support for improved motor control.
    - REV Robotics Expansion Hub firmware 1.8 and greater will support a feed forward mechanism for closed loop motor control.
    - FTC SDK has been modified to support PIDF coefficients (proportional, integral, derivative, and feed forward).
    - FTC Blocks development tool modified to include PIDF programming blocks.
    - Deprecated older PID-related methods and variables.
    - REV's 1.8.x PIDF-related changes provide a more linear and accurate way to control a motor.

 * Wireless
    - Added 5GHz support for wireless channel changing for those devices that support it.
        + Tested with Moto G5 and E4 phones.
	+ Also tested with other (currently non-approved) phones such as Samsung Galaxy S8.

* Improved Expansion Hub firmware update support in Robot Controller app
    - Changes to make the system more robust during the firmware update process (when performed through Robot Controller app).
    - User no longer has to disconnect a downstream daisy-chained Expansion Hub when updating an Expansion Hub's firmware.
        + If user is updating an Expansion Hub's firmware through a USB connection, he/she does not have to disconnect RS485 connection to other Expansion Hubs.
	+ The user still must use a USB connection to update an Expansion Hub's firmware.
	+ The user cannot update the Expansion Hub firmware for a downstream device that is daisy chained through an RS485 connection.
    - If an Expansion Hub accidentally gets "bricked" the Robot Controller app is now more likely to recognize the Hub when it scans the USB bus.
        + Robot Controller app should be able to detect an Expansion Hub, even if it accidentally was bricked in a previous update attempt.
	+ Robot Controller app should be able to install the firmware onto the Hub, even if if accidentally was bricked in a previous update attempt.

 * Resiliency
    - FTC software can detect and enable an FTDI reset feature that is available with REV Robotics v1.8 Expansion Hub firmware and greater.
        + When enabled, the Expansion Hub can detect if it hasn't communicated with the Robot Controller over the FTDI (USB) connection.
	+ If the Hub hasn't heard from the Robot Controller in a while, it will reset the FTDI connection.
	+ This action helps system recover from some ESD-induced disruptions.
    - Various fixes to improve reliability of FTC software.

 * Blocks
    - Fixed errors with string and list indices in blocks export to java.
    - Support for USB connected UVC webcams.
    - Refactored optimized Blocks Vuforia code to support Rover Ruckus image targets.
    - Added programming blocks to support PIDF (proportional, integral, derivative and feed forward) motor control.
    - Added formatting options (under Telemetry and Miscellaneous categories) so user can set how many decimal places to display a numerical value.
    - Support to play audio files (which are uploaded through Blocks web interface) on Driver Station in addition to the Robot Controller.
    - Fixed bug with Download Image of Blocks feature.
    - Support for REV Robotics Blinkin LED Controller.
    - Support for REV Robotics 2m Distance Sensor.
    - Added support for a REV Touch Sensor (no longer have to configure as a generic digital device).
    - Added blocks for DcMotorEx methods.
        + These are enhanced methods that you can use when supported by the motor controller hardware.
	+ The REV Robotics Expansion Hub supports these enhanced methods.
	+ Enhanced methods include methods to get/set motor velocity (in encoder pulses per second), get/set PIDF coefficients, etc..

 * Modest Improvements in Logging
    - Decrease frequency of battery checker voltage statements.
    - Removed non-FTC related log statements (wherever possible).
    - Introduced a "Match Logging" feature.
        + Under "Settings" a user can enable/disable this feature (it's disabled by default).
	+ If enabled, user provides a "Match Number" through the Driver Station user interface (top of the screen).
	    * The Match Number is used to create a log file specifically with log statements from that particular OpMode run.
	    * Match log files are stored in /sdcard/FIRST/matlogs on the Robot Controller.
	    * Once an OpMode run is complete, the Match Number is cleared.
	    * This is a convenient way to create a separate match log with statements only related to a specific OpMode run.

 * New Devices
    - Support for REV Robotics Blinkin LED Controller.
    - Support for REV Robotics 2m Distance Sensor.
    - Added configuration option for REV 20:1 HD Hex Motor.
    - Added support for a REV Touch Sensor (no longer have to configure as a generic digital device).

 * Miscellaneous
    - Fixed some errors in the definitions for acceleration and velocity in our javadoc documentation.
    - Added ability to play audio files on Driver Station
    - When user is configuring an Expansion Hub, the LED on the Expansion Hub will change blink pattern (purple-cyan)  to indicate which Hub is currently being configured.
    - Renamed I2cSensorType to I2cDeviceType.
    - Added an external sample OpMode that demonstrates localization using 2018-2019 (Rover Ruckus presented by QualComm) Vuforia targets.
    - Added an external sample OpMode that demonstrates how to use the REV Robotics 2m Laser Distance Sensor.
    - Added an external sample OpMode that demonstrates how to use the REV Robotics Blinkin LED Controller.
    - Re-categorized external Java sample OpModes to "TeleOp" instead of "Autonomous".

Known issues:
 * Initial support for UVC compatible cameras
    - UVC cameras seem to draw significant amount of current from the USB bus.
        + This does not appear to present any problems for the REV Robotics Control Hub.
	+ This does seem to create stability problems when using some cameras with an Android phone-based Robot Controller.
	+ FTC Tech Team is investigating options to mitigate this issue with the phone-based Robot Controllers.
    - There might be a possible deadlock which causes the RC to become unresponsive when using a UVC webcam with a Nougat Android Robot Controller.

 * Wireless
    - When user selects a wireless channel, this channel does not necessarily persist if the phone is power cycled.
        + Tech Team is hoping to eventually address this issue in a future release.
	+ Issue has been present since apps were introduced (i.e., it is not new with the v4.0 release).
    - Wireless channel is not currently displayed for Wi-Fi Direct connections.

 * Miscellaneous
    - The blink indication feature that shows which Expansion Hub is currently being configured does not work for a newly created configuration file.
        + User has to first save a newly created configuration file and then close and re-edit the file in order for blink indicator to work.

## Version 3.6 (built on 17.12.18)

Changes include:
 * Blocks Changes
     - Uses updated Google Blockly software to allow users to edit their OpModes on Apple iOS devices (including iPad and iPhone).
     - Improvement in Blocks tool to handle corrupt OpMode files.
     - Autonomous OpModes should no longer get switched back to tele-op after re-opening them to be edited.
     - The system can now detect type mismatches during runtime and alert the user with a message on the Driver Station.
 * Updated javadoc documentation for setPower() method to reflect correct range of values (-1 to +1).
 * Modified VuforiaLocalizerImpl to allow for user rendering of frames
     - Added a user-overrideable onRenderFrame() method which gets called by the class's renderFrame() method.

## Version 3.5 (built on 17.10.30)

Changes with version 3.5 include:
 * Introduced a fix to prevent random OpMode stops, which can occur after the Robot Controller app has been paused and then resumed (for example, when a user temporarily turns off the display of the Robot Controller phone, and then turns the screen back on).
 * Introduced a fix to prevent random OpMode stops, which were previously caused by random peer disconnect events on the Driver Station.
 * Fixes issue where log files would be closed on pause of the RC or DS, but not re-opened upon resume.
 * Fixes issue with battery handler (voltage) start/stop race.
 * Fixes issue where Android Studio generated OpModes would disappear from available list in certain situations.
 * Fixes problem where OnBot Java would not build on REV Robotics Control Hub.
 * Fixes problem where OnBot Java would not build if the date and time on the Robot Controller device was "rewound" (set to an earlier date/time).
 * Improved error message on OnBot Java that occurs when renaming a file fails.
 * Removed unneeded resources from android.jar binaries used by OnBot Java to reduce final size of Robot Controller app.
 * Added MR_ANALOG_TOUCH_SENSOR block to Blocks Programming Tool.

## Version 3.4 (built on 17.09.06)

Changes with version 3.4 include:
 * Added telemetry.update() statement for BlankLinearOpMode template.
 * Renamed sample Block OpModes to be more consistent with Java samples.
 * Added some additional sample Block OpModes.
 * Reworded OnBot Java readme slightly.

## Version 3.3 (built on 17.09.04)

This version of the software includes improves for the FTC Blocks Programming Tool and the OnBot Java Programming Tool.

Changes with verion 3.3 include:
 * Android Studio ftc_app project has been updated to use Gradle Plugin 2.3.3.
 * Android Studio ftc_app project is already using gradle 3.5 distribution.
 * Robot Controller log has been renamed to /sdcard/RobotControllerLog.txt (note that this change was actually introduced w/ v3.2).
 * Improvements in I2C reliability.
 * Optimized I2C read for REV Expansion Hub, with v1.7 firmware or greater.
 * Updated all external/samples (available through OnBot and in Android project folder).
 * Vuforia
    - Added support for VuMarks that will be used for the 2017-2018 season game.
 * Blocks
    - Update to latest Google Blockly release.
    - Sample OpModes can be selected as a template when creating new OpMode.
    - Fixed bug where the blocks would disappear temporarily when mouse button is held down.
    - Added blocks for Range.clip and Range.scale.
    - User can now disable/enable Block OpModes.
    - Fix to prevent occasional Blocks deadlock.
 * OnBot Java
    - Significant improvements with autocomplete function for OnBot Java editor.
    - Sample OpModes can be selected as a template when creating new OpMode.
    - Fixes and changes to complete hardware setup feature.
    - Updated (and more useful) onBot welcome message.

Known issues:
 * Android Studio
    - After updating to the new v3.3 Android Studio project folder, if you get error messages indicating "InvalidVirtualFileAccessException" then you might need to do a File->Invalidate Caches / Restart to clear the error.
 * OnBot Java
    - Sometimes when you push the build button to build all OpModes, the RC returns an error message that the build failed.  If you press the build button a second time, the build typically suceeds.

## Version 3.2 (built on 17.08.02)

This version of the software introduces the "OnBot Java" Development Tool.  Similar to the FTC Blocks Development Tool, the FTC OnBot Java Development Tool allows a user to create, edit and build OpModes dynamically using only a Javascript-enabled web browser.

The OnBot Java Development Tool is an integrated development environment (IDE) that is served up by the Robot Controller.  OpModes are created and edited using a Javascript-enabled browser (Google Chromse is recommended).  OpModes are saved on the Robot Controller Android device directly.

The OnBot Java Development Tool provides a Java programming environment that does NOT need Android Studio.



Changes with version 3.2 include:
 * Enhanced web-based development tools
    - Introduction of OnBot Java Development Tool.
    - Web-based programming and management features are "always on" (user no longer needs to put Robot Controller into programming mode).
    - Web-based management interface (where user can change Robot Controller name and also easily download Robot Controller log file).
    - OnBot Java, Blocks and Management features available from web based interface.

* Blocks Programming Development Tool:
    - Changed "LynxI2cColorRangeSensor" block to "REV Color/range sensor" block.
    - Fixed tooltip for ColorSensor.isLightOn block.
    Added blocks for ColorSensor.getNormalizedColors and LynxI2cColorRangeSensor.getNormalizedColors.

* Added example OpModes for digital touch sensor and REV Robotics Color Distance sensor.
* User selectable color themes.
* Includes many minor enhancements and fixes (too numerous to list).

Known issues:
* Auto complete function is incomplete and does not support the following (for now):
     - Access via *this* keyword
     - Access via *super* keyword
     - Members of the super cloass, not overridden by the class
     - Any methods provided in the current class
     - Inner classes
     - Can't handle casted objects
     - Any objects coming from an parenthetically enclosed expression

## Version 3.10 (built on 17.05.09)

This version of the software provides support for the REV Robotics Expansion Hub.  This version also includes improvements in the USB communication layer in an effort to enhance system resiliency.  If you were using a 2.x version of the software previously, updating to version 3.1 requires that you also update your Driver Station software in addition to updating the Robot Controller software.

Also note that in version 3.10 software, the setMaxSpeed and getMaxSpeed methods are no longer available (not deprecated, they have been removed from the SDK). Also note that the the new 3.x software incorporates motor profiles that a user can select as he/she configures the robot.

Changes include:
 * Blocks changes
    - Added VuforiaTrackableDefaultListener.getPose and Vuforia.trackPose blocks.
    - Added optimized blocks support for Vuforia extended tracking.
    - Added atan2 block to the math category.
    - Added useCompetitionFieldTargetLocations parameter to Vuforia.initialize block.  If set to false, the target locations are placed at (0,0,0) with target orientation as specified in https://github.com/gearsincorg/FTCVuforiaDemo/blob/master/Robot_Navigation.java tutorial OpMode.
 * Incorporates additional improvements to USB comm layer to improve system resiliency (to recover from a greater number of communication disruptions).

**************************************************************************************

Additional Notes Regarding Version 3.00 (built on 17.04.13)

In addition to the release changes listed below (see section labeled "Version 3.00 (built on 17.04.013)"), version 3.00 has the following important changes:

1. Version 3.00 software uses a new version of the FTC Robocol (robot protocol).  If you upgrade to v3.0 on the Robot Controller and/or Android Studio side, you must also upgrade the Driver Station software to match the new Robocol.
2. Version 3.00 software removes the setMaxSpeed and getMaxSpeed methods from the DcMotor class.  If you have an OpMode that formerly used these methods, you will need to remove the references/calls to these methods.  Instead, v3.0 provides the max speed information through the use of motor profiles that are selected by the user during robot configuration.
3. Version 3.00 software currently does not have a mechanism to disable extra i2c sensors.  We hope to re-introduce this function with a release in the near future.

**************************************************************************************

## Version 3.00 (built on 17.04.13)

*** Use this version of the software at YOUR OWN RISK!!! ***

This software is being released as an "alpha" version.  Use this version at your own risk!

This pre-release software contains SIGNIFICANT changes, including changes to the Wi-Fi Direct pairing mechanism, rewrites of the I2C sensor classes, changes to the USB/FTDI layer, and the introduction of support for the REV Robotics Expansion Hub and the REV Robotics color-range-light sensor.  These changes were implemented to improve the reliability and resiliency of the FTC control system.

Please note, however, that version 3.00 is considered "alpha" code.  This code is being released so that the FIRST community will have an opportunity to test the new REV Expansion Hub electronics module when it becomes available in May.  The developers do not recommend using this code for critical applications (i.e., competition use).

*** Use this version of the software at YOUR OWN RISK!!! ***

Changes include:
 * Major rework of sensor-related infrastructure.  Includes rewriting sensor classes to implement synchronous I2C communication.
 * Fix to reset Autonomous timer back to 30 seconds.
 * Implementation of specific motor profiles for approved 12V motors (includes Tetrix, AndyMark, Matrix and REV models).
 * Modest improvements to enhance Wi-Fi P2P pairing.
 * Fixes telemetry log addition race.
 * Publishes all the sources (not just a select few).
 * Includes Block programming improvements
    - Addition of optimized Vuforia blocks.
    - Auto scrollbar to projects and sounds pages.
    - Fixed blocks paste bug.
    - Blocks execute after while-opModeIsActive loop (to allow for cleanup before exiting OpMode).
    - Added gyro integratedZValue block.
    - Fixes bug with projects page for Firefox browser.
    - Added IsSpeaking block to AndroidTextToSpeech.
 * Implements support for the REV Robotics Expansion Hub
    - Implements support for integral REV IMU (physically installed on I2C bus 0, uses same Bosch BNO055 9 axis absolute orientation sensor as Adafruit 9DOF abs orientation sensor).    - Implements support for REV color/range/light sensor.
    - Provides support to update Expansion Hub firmware through FTC SDK.
    - Detects REV firmware version and records in log file.
    - Includes support for REV Control Hub (note that the REV Control Hub is not yet approved for FTC use).
    - Implements FTC Blocks programming support for REV Expansion Hub and sensor hardware.
    - Detects and alerts when I2C device disconnect.

## Version 2.62 (built on 17.01.07)
  * Added null pointer check before calling modeToByte() in finishModeSwitchIfNecessary method for ModernRoboticsUsbDcMotorController class.
  * Changes to enhance Modern Robotics USB protocol robustness.

## Version 2.61 (released on 16.12.19)
  * Blocks Programming mode changes:
     - Fix to correct issue when an exception was thrown because an OpticalDistanceSensor object appears twice in the hardware map (the second time as a LightSensor).

## Version 2.6 (released on 16.12.16)
  * Fixes for Gyro class:
     - Improve (decrease) sensor refresh latency.
     - fix isCalibrating issues.
  * Blocks Programming mode changes:
     - Blocks now ignores a device in the configuration xml if the name is empty. Other devices work in configuration work fine.

## Version 2.5 (internal release on released on 16.12.13)
  * Blocks Programming mode changes:
     - Added blocks support for AdafruitBNO055IMU.
     - Added Download OpMode button to FtcBocks.html.
     - Added support for copying blocks in one OpMode and pasting them in an other OpMode. The clipboard content is stored on the phone, so the programming mode server must be running.
     - Modified Utilities section of the toolbox.
     - In Programming Mode, display information about the active connections.
     - Fixed paste location when workspace has been scrolled.
     - Added blocks support for the android Accelerometer.
     - Fixed issue where Blocks Upload OpMode truncated name at first dot.
     - Added blocks support for Android SoundPool.
     - Added type safety to blocks for Acceleration.
     - Added type safety to blocks for AdafruitBNO055IMU.Parameters.
     - Added type safety to blocks for AnalogInput.
     - Added type safety to blocks for AngularVelocity.
     - Added type safety to blocks for Color.
     - Added type safety to blocks for ColorSensor.
     - Added type safety to blocks for CompassSensor.
     - Added type safety to blocks for CRServo.
     - Added type safety to blocks for DigitalChannel.
     - Added type safety to blocks for ElapsedTime.
     - Added type safety to blocks for Gamepad.
     - Added type safety to blocks for GyroSensor.
     - Added type safety to blocks for IrSeekerSensor.
     - Added type safety to blocks for LED.
     - Added type safety to blocks for LightSensor.
     - Added type safety to blocks for LinearOpMode.
     - Added type safety to blocks for MagneticFlux.
     - Added type safety to blocks for MatrixF.
     - Added type safety to blocks for MrI2cCompassSensor.
     - Added type safety to blocks for MrI2cRangeSensor.
     - Added type safety to blocks for OpticalDistanceSensor.
     - Added type safety to blocks for Orientation.
     - Added type safety to blocks for Position.
     - Added type safety to blocks for Quaternion.
     - Added type safety to blocks for Servo.
     - Added type safety to blocks for ServoController.
     - Added type safety to blocks for Telemetry.
     - Added type safety to blocks for Temperature.
     - Added type safety to blocks for TouchSensor.
     - Added type safety to blocks for UltrasonicSensor.
     - Added type safety to blocks for VectorF.
     - Added type safety to blocks for Velocity.
     - Added type safety to blocks for VoltageSensor.
     - Added type safety to blocks for VuforiaLocalizer.Parameters.
     - Added type safety to blocks for VuforiaTrackable.
     - Added type safety to blocks for VuforiaTrackables.
     - Added type safety to blocks for enums in AdafruitBNO055IMU.Parameters.
     - Added type safety to blocks for AndroidAccelerometer, AndroidGyroscope, AndroidOrientation, and AndroidTextToSpeech.

## Version 2.4 (released on 16.11.13)
  * Fix to avoid crashing for nonexistent resources.
  * Blocks Programming mode changes:
     - Added blocks to support OpenGLMatrix, MatrixF, and VectorF.
     - Added blocks to support AngleUnit, AxesOrder, AxesReference, CameraDirection, CameraMonitorFeedback, DistanceUnit, and TempUnit.
     - Added blocks to support Acceleration.
     - Added blocks to support LinearOpMode.getRuntime.
     - Added blocks to support MagneticFlux and Position.
     - Fixed typos.
     - Made blocks for ElapsedTime more consistent with other objects.
     - Added blocks to support Quaternion, Velocity, Orientation, AngularVelocity.
     - Added blocks to support VuforiaTrackables, VuforiaTrackable, VuforiaLocalizer, VuforiaTrackableDefaultListener.
     - Fixed a few blocks.
     - Added type checking to new blocks.
     - Updated to latest blockly.
     - Added default variable blocks to navigation and matrix blocks.
     - Fixed toolbox entry for openGLMatrix_rotation_withAxesArgs.
     - When user downloads Blocks-generated OpMode, only the .blk file is downloaded.
     - When user uploads Blocks-generated OpMode (.blk file), Javascript code is auto generated.
     - Added DbgLog support.
     - Added logging when a blocks file is read/written.
     - Fixed bug to properly render blocks even if missing devices from configuration file.
     - Added support for additional characters (not just alphanumeric) for the block file names (for download and upload).
     - Added support for OpMode flavor (“Autonomous” or “TeleOp”) and group.
  * Changes to Samples to prevent tutorial issues.
  * Incorporated suggested changes from public pull 216 (“Replace .. paths”).
  * Remove Servo Glitches when robot stopped.
  * if user hits “Cancels” when editing a configuration file, clears the unsaved changes and reverts to original unmodified configuration.
  * Added log info to help diagnose why the Robot Controller app was terminated (for example, by watch dog function).
  * Added ability to transfer log from the controller.
  * Fixed inconsistency for AngularVelocity
  * Limit unbounded growth of data for telemetry.  If user does not call telemetry.update() for LinearOpMode in a timely manner, data added for telemetry might get lost if size limit is exceeded.

## Version 2.35 (released on 16.10.06)
  * Blockly programming mode - Removed unnecesary idle() call from blocks for new project.

## Version 2.30 (released on 16.10.05)
  * Blockly programming mode:
     - Mechanism added to save Blockly OpModes from Programming Mode Server onto local device
     - To avoid clutter, blocks are displayed in categorized folders
     - Added support for DigitalChannel
     - Added support for ModernRoboticsI2cCompassSensor
     - Added support for ModernRoboticsI2cRangeSensor
     - Added support for VoltageSensor
     - Added support for AnalogInput
     - Added support for AnalogOutput
     - Fix for CompassSensor setMode block
  * Vuforia
     - Fix deadlock / make camera data available while Vuforia is running.
     - Update to Vuforia 6.0.117 (recommended by Vuforia and Google to close security loophole).
  * Fix for autonomous 30 second timer bug (where timer was in effect, even though it appeared to have timed out).
  * opModeIsActive changes to allow cleanup after OpMode is stopped (with enforced 2 second safety timeout).
  * Fix to avoid reading i2c twice.
  * Updated sample OpModes.
  * Improved logging and fixed intermittent freezing.
  * Added digital I/O sample.
  * Cleaned up device names in sample OpModes to be consistent with Pushbot guide.
  * Fix to allow use of IrSeekerSensorV3.

## Version 2.20 (released on 16.09.08)
  * Support for Modern Robotics Compass Sensor.
  * Support for Modern Robotics Range Sensor.
  * Revise device names for Pushbot templates to match the names used in Pushbot guide.
  * Fixed bug so that IrSeekerSensorV3 device is accessible as IrSeekerSensor in hardwareMap.
  * Modified computer vision code to require an individual Vuforia license (per legal requirement from PTC).
  * Minor fixes.
  * Blockly enhancements:
     - Support for Voltage Sensor.
     - Support for Analog Input.
     - Support for Analog Output.
     - Support for Light Sensor.
     - Support for Servo Controller.

## Version 2.10 (released on 16.09.03)
 * Support for Adafruit IMU.
 * Improvements to ModernRoboticsI2cGyro class
    - Block on reset of z axis.
    - isCalibrating() returns true while gyro is calibration.
 * Updated sample gyro program.
 * Blockly enhancements
    - support for android.graphics.Color.
    - added support for ElapsedTime.
    - improved look and legibility of blocks.
    - support for compass sensor.
    - support for ultrasonic sensor.
    - support for IrSeeker.
    - support for LED.
    - support for color sensor.
    - support for CRServo
    - prompt user to configure robot before using programming mode.
 * Provides ability to disable audio cues.
 * various bug fixes and improvements.

## Version 2.00 (released on 16.08.19)
 * This is the new release for the upcoming 2016-2017 FIRST Tech Challenge Season.
 * Channel change is enabled in the FTC Robot Controller app for Moto G 2nd and 3rd Gen phones.
 * Users can now use annotations to register/disable their OpModes.
 * Changes in the Android SDK, JDK and build tool requirements (minsdk=19, java 1.7, build tools 23.0.3).
 * Standardized units in analog input.
 * Cleaned up code for existing analog sensor classes.
 * setChannelMode and getChannelMode were REMOVED from the DcMotorController class.  This is important - we no longer set the motor modes through the motor controller.
 * setMode and getMode were added to the DcMotor class.
 * ContinuousRotationServo class has been added to the FTC SDK.
 * Range.clip() method has been overloaded so it can support this operation for int, short and byte integers.
 * Some changes have been made (new methods added) on how a user can access items from the hardware map.
 * Users can now set the zero power behavior for a DC motor so that the motor will brake or float when power is zero.
 * Prototype Blockly Programming Mode has been added to FTC Robot Controller.  Users can place the Robot Controller into this mode, and then use a device (such as a laptop) that has a Javascript enabled browser to write Blockly-based OpModes directly onto the Robot Controller.
 * Users can now configure the robot remotely through the FTC Driver Station app.
 * Android Studio project supports Android Studio 2.1.x and compile SDK Version 23 (Marshmallow).
 * Vuforia Computer Vision SDK integrated into FTC SDK.  Users can use sample vision targets to get localization information on a standard FTC field.
 * Project structure has been reorganized so that there is now a TeamCode package that users can use to place their local/custom OpModes into this package.
 * Inspection function has been integrated into the FTC Robot Controller and Driver Station Apps (Thanks Team HazMat… 9277 & 10650!).
 * Audio cues have been incorporated into FTC SDK.
 * Swap mechanism added to FTC Robot Controller configuration activity.  For example, if you have two motor controllers on a robot, and you misidentified them in your configuration file, you can use the Swap button to swap the devices within the configuration file (so you do not have to manually re-enter in the configuration info for the two devices).
 * Fix mechanism added to all user to replace an electronic module easily.  For example, suppose a servo controller dies on your robot. You replace the broken module with a new module, which has a different serial number from the original servo controller.  You can use the Fix button to automatically reconfigure your configuration file to use the serial number of the new module.
 * Improvements made to fix resiliency and responsiveness of the system.
 * For LinearOpMode the user now must for a telemetry.update() to update the telemetry data on the driver station.  This update() mechanism ensures that the driver station gets the updated data properly and at the same time.
 * The Auto Configure function of the Robot Controller is now template based.  If there is a commonly used robot configuration, a template can be created so that the Auto Configure mechanism can be used to quickly configure a robot of this type.
 * The logic to detect a runaway OpMode (both in the LinearOpMode and OpMode types) and to abort the run, then auto recover has been improved/implemented.
 * Fix has been incorporated so that Logitech F310 gamepad mappings will be correct for Marshmallow users.

## Release 16.07.08

 * For the ftc_app project, the gradle files have been modified to support Android Studio 2.1.x.

## Release 16.03.30

 * For the MIT App Inventor, the design blocks have new icons that better represent the function of each design component.
 * Some changes were made to the shutdown logic to ensure the robust shutdown of some of our USB services.
 * A change was made to LinearOpMode so as to allow a given instance to be executed more than once, which is required for the App Inventor.
 * Javadoc improved/updated.

## Release 16.03.09

 * Changes made to make the FTC SDK synchronous (significant change!)
    - waitOneFullHardwareCycle() and waitForNextHardwareCycle() are no longer needed and have been deprecated.
    - runOpMode() (for a LinearOpMode) is now decoupled from the system's hardware read/write thread.
    - loop() (for an OpMode) is now decoupled from the system's hardware read/write thread.
    - Methods are synchronous.
    - For example, if you call setMode(DcMotorController.RunMode.RESET_ENCODERS) for a motor, the encoder is guaranteed to be reset when the method call is complete.
    - For legacy module (NXT compatible), user no longer has to toggle between read and write modes when reading from or writing to a legacy device.
 * Changes made to enhance reliability/robustness during ESD event.
 * Changes made to make code thread safe.
 * Debug keystore added so that user-generated robot controller APKs will all use the same signed key (to avoid conflicts if a team has multiple developer laptops for example).
 * Firmware version information for Modern Robotics modules are now logged.
 * Changes made to improve USB comm reliability and robustness.
 * Added support for voltage indicator for legacy (NXT-compatible) motor controllers.
 * Changes made to provide auto stop capabilities for OpModes.
    - A LinearOpMode class will stop when the statements in runOpMode() are complete.  User does not have to push the stop button on the driver station.
    - If an OpMode is stopped by the driver station, but there is a run away/uninterruptible thread persisting, the app will log an error message then force itself to crash to stop the runaway thread.
 * Driver Station UI modified to display lowest measured voltage below current voltage (12V battery).
 * Driver Station UI modified to have color background for current voltage (green=good, yellow=caution, red=danger, extremely low voltage).
 * javadoc improved (edits and additional classes).
 * Added app build time to About activity for driver station and robot controller apps.
 * Display local IP addresses on Driver Station About activity.
 * Added I2cDeviceSynchImpl.
 * Added I2cDeviceSync interface.
 * Added seconds() and milliseconds() to ElapsedTime for clarity.
 * Added getCallbackCount() to I2cDevice.
 * Added missing clearI2cPortActionFlag.
 * Added code to create log messages while waiting for LinearOpMode shutdown.
 * Fix so Wi-Fi Direct Config activity will no longer launch multiple times.
 * Added the ability to specify an alternate i2c address in software for the Modern Robotics gyro.

## Release 16.02.09

 * Improved battery checker feature so that voltage values get refreshed regularly (every 250 msec) on Driver Station (DS) user interface.
 * Improved software so that Robot Controller (RC) is much more resilient and “self-healing” to USB disconnects:
    - If user attempts to start/restart RC with one or more module missing, it will display a warning but still start up.
    - When running an OpMode, if one or more modules gets disconnected, the RC & DS will display warnings,and robot will keep on working in spite of the missing module(s).
    - If a disconnected module gets physically reconnected the RC will auto detect the module and the user will regain control of the recently connected module.
    - Warning messages are more helpful (identifies the type of module that’s missing plus its USB serial number).
 * Code changes to fix the null gamepad reference when users try to reference the gamepads in the init() portion of their OpMode.
 * NXT light sensor output is now properly scaled.  Note that teams might have to readjust their light threshold values in their OpModes.
 * On DS user interface, gamepad icon for a driver will disappear if the matching gamepad is disconnected or if that gamepad gets designated as a different driver.
 * Robot Protocol (ROBOCOL) version number info is displayed in About screen on RC and DS apps.
 * Incorporated a display filter on pairing screen to filter out devices that don’t use the “<TEAM NUMBER>-“ format. This filter can be turned off to show all Wi-Fi Direct devices.
 * Updated text in License file.
 * Fixed formatting error in OpticalDistanceSensor.toString().
 * Fixed issue on with a blank (“”) device name that would disrupt Wi-Fi Direct Pairing.
 * Made a change so that the Wi-Fi info and battery info can be displayed more quickly on the DS upon connecting to RC.
 * Improved javadoc generation.
 * Modified code to make it easier to support language localization in the future.

## Release 16.01.04

 * Updated compileSdkVersion for apps
 * Prevent Wi-Fi from entering power saving mode
 * removed unused import from driver station
 * Corrrected "Dead zone" joystick code.
 * LED.getDeviceName and .getConnectionInfo() return null
 * apps check for ROBOCOL_VERSION mismatch
 * Fix for Telemetry also has off-by-one errors in its data string sizing / short size limitations error
 * User telemetry output is sorted.
 * added formatting variants to DbgLog and RobotLog APIs
 * code modified to allow for a long list of OpMode names.
 * changes to improve thread safety of RobocolDatagramSocket
 * Fix for "missing hardware leaves robot controller disconnected from driver station" error
 * fix for "fast tapping of Init/Start causes problems" (toast is now only instantiated on UI thread).
 * added some log statements for thread life cycle.
 * moved gamepad reset logic inside of initActiveOpMode() for robustness
 * changes made to mitigate risk of race conditions on public methods.
 * changes to try and flag when Wi-Fi Direct name contains non-printable characters.
 * fix to correct race condition between .run() and .close() in ReadWriteRunnableStandard.
 * updated FTDI driver
 * made ReadWriteRunnableStanard interface public.
 * fixed off-by-one errors in Command constructor
 * moved specific hardware implmentations into their own package.
 * moved specific gamepad implemnatations to the hardware library.
 * changed LICENSE file to new BSD version.
 * fixed race condition when shutting down Modern Robotics USB devices.
 * methods in the ColorSensor classes have been synchronized.
 * corrected isBusy() status to reflect end of motion.
 * corrected "back" button keycode.
 * the notSupported() method of the GyroSensor class was changed to protected (it should not be public).

## Release 15.11.04.001

 * Added Support for Modern Robotics Gyro.
  - The GyroSensor class now supports the MR Gyro Sensor.
  - Users can access heading data (about Z axis)
  - Users can also access raw gyro data (X, Y, & Z axes).
  - Example MRGyroTest.java OpMode included.
 * Improved error messages
  - More descriptive error messages for exceptions in user code.
 * Updated DcMotor API
 * Enable read mode on new address in setI2cAddress
 * Fix so that driver station app resets the gamepads when switching OpModes.
 * USB-related code changes to make USB comm more responsive and to display more explicit error messages.
  - Fix so that USB will recover properly if the USB bus returns garbage data.
  - Fix USB initializtion race condition.
  - Better error reporting during FTDI open.
  - More explicit messages during USB failures.
  - Fixed bug so that USB device is closed if event loop teardown method was not called.
 * Fixed timer UI issue
 * Fixed duplicate name UI bug (Legacy Module configuration).
 * Fixed race condition in EventLoopManager.
 * Fix to keep references stable when updating gamepad.
 * For legacy Matrix motor/servo controllers removed necessity of appending "Motor" and "Servo" to controller names.
 * Updated HT color sensor driver to use constants from ModernRoboticsUsbLegacyModule class.
 * Updated MR color sensor driver to use constants from ModernRoboticsUsbDeviceInterfaceModule class.
 * Correctly handle I2C Address change in all color sensors
 * Updated/cleaned up OpModes.
  - Updated comments in LinearI2cAddressChange.java example OpMode.
  - Replaced the calls to "setChannelMode" with "setMode" (to match the new of the DcMotor  method).
  - Removed K9AutoTime.java OpMode.
  - Added MRGyroTest.java OpMode (demonstrates how to use MR Gyro Sensor).
  - Added MRRGBExample.java OpMode (demonstrates how to use MR Color Sensor).
  - Added HTRGBExample.java OpMode (demonstrates how to use HT legacy color sensor).
  - Added MatrixControllerDemo.java (demonstrates how to use legacy Matrix controller).
 * Updated javadoc documentation.
 * Updated release .apk files for Robot Controller and Driver Station apps.

## Release 15.10.06.002

 * Added support for Legacy Matrix 9.6V motor/servo controller.
 * Cleaned up build.gradle file.
 * Minor UI and bug fixes for driver station and robot controller apps.
 * Throws error if Ultrasonic sensor (NXT) is not configured for legacy module port 4 or 5.


## Release 15.08.03.001

 * New user interfaces for FTC Driver Station and FTC Robot Controller apps.
 * An init() method is added to the OpMode class.
   - For this release, init() is triggered right before the start() method.
   - Eventually, the init() method will be triggered when the user presses an "INIT" button on driver station.
   - The init() and loop() methods are now required (i.e., need to be overridden in the user's OpMode).
   - The start() and stop() methods are optional.
 * A new LinearOpMode class is introduced.
   - Teams can use the LinearOpMode mode to create a linear (not event driven) program model.
   - Teams can use blocking statements like Thread.sleep() within a linear OpMode.
 * The API for the Legacy Module and Core Device Interface Module have been updated.
   - Support for encoders with the Legacy Module is now working.
 * The hardware loop has been updated for better performance.
