/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import androidx.annotation.NonNull;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.motor.FtcDcMotor;
import ftclib.motor.FtcServo;
import ftclib.robotcore.FtcOpMode;
import ftclib.sensor.FtcRobotBattery;
import teamcode.subsystems.Arm;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.subsystems.Elevator;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Intake;
import teamcode.subsystems.RobotBase;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcDiscreteValue;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.subsystem.TrcIntake;
import trclib.subsystem.TrcServoGrabber;
import trclib.subsystem.TrcShooter;
import trclib.timer.TrcTimer;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private final String moduleName = getClass().getSimpleName();
    // Global objects.
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    private static double nextStatusUpdateTime = 0.0;
    // Robot Drive.
    public FtcRobotDrive.RobotInfo robotInfo;
    public FtcRobotDrive robotDrive;
    // Vision subsystems.
    public Vision vision;
    // Sensors and indicators.
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;
    // Subsystems.
    public FtcDcMotor simpleMotor;
    public FtcServo simpleServo;
    public TrcMotor elevator;
    public TrcMotor arm;
    public TrcShooter shooter;
    public TrcDiscreteValue shooterVelocity;
    public TrcIntake intake;
    public TrcServoGrabber grabber;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects.
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        nextStatusUpdateTime = TrcTimer.getCurrentTime();
        speak("Init starting");
        // Create and initialize Robot Base.
        RobotBase robotBase = new RobotBase();
        robotInfo = robotBase.getRobotInfo();
        robotDrive = robotBase.getRobotDrive();
        // Create and initialize vision subsystems.
        if (RobotParams.Preferences.useVision &&
            (RobotParams.Preferences.tuneColorBlobVision ||
             RobotParams.Preferences.useAprilTagVision ||
             RobotParams.Preferences.useColorBlobVision ||
             RobotParams.Preferences.useLimelightVision))
        {
            vision = new Vision(this);
        }
        // If robotType is VisionOnly, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != RobotParams.RobotType.VisionOnly)
        {
            // Create and initialize sensors and indicators.
            if (robotInfo.blinkinName != null)
            {
                blinkin = new BlinkinLEDs(robotInfo.blinkinName);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useSimpleMotor)
                {
                    final double goBilda1620CPR = ((1.0 + (46.0/17.0)) * 28.0);
                    simpleMotor = new FtcDcMotor("SimpleMotor");
                    simpleMotor.resetFactoryDefault();
                    simpleMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
                    simpleMotor.setBrakeModeEnabled(true);
                    simpleMotor.setMotorInverted(true);
                    simpleMotor.setPositionSensorScaleAndOffset(1.0 / goBilda1620CPR, 0.0);
                }

                if (RobotParams.Preferences.useSimpleServo)
                {
                    simpleServo = new FtcServo("SimpleServo");
                    simpleServo.setLogicalPosRange(0.2, 0.6);
                    simpleServo.setPhysicalPosRange(0.0, 90.0);
                    simpleServo.setMaxStepRate(250.0);
                    simpleServo.setPosition(0.0);   // in degrees
                }

                if (RobotParams.Preferences.useElevator)
                {
                    elevator = new Elevator().getElevatorMotor();
                }

                if (RobotParams.Preferences.useArm)
                {
                    arm = new Arm().getArmMotor();
                }

                if (RobotParams.Preferences.useShooter)
                {
                    shooter = new Shooter().getShooter();
                    shooterVelocity = new TrcDiscreteValue(
                        RobotParams.Shooter.SUBSYSTEM_NAME + ".motorVel",
                        RobotParams.Shooter.SHOOTER_MIN_VEL, RobotParams.Shooter.SHOOTER_MAX_VEL,
                        RobotParams.Shooter.SHOOTER_MIN_VEL_INC, RobotParams.Shooter.SHOOTER_MAX_VEL_INC,
                        RobotParams.Shooter.SHOOTER_DEF_VEL, RobotParams.Shooter.SHOOTER_DEF_VEL_INC);
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake().getIntake();
                }

                if (RobotParams.Preferences.useGrabber)
                {
                    grabber = new Grabber().getGrabber();
                }
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return robotInfo != null? robotInfo.robotName: RobotParams.Robot.ROBOT_CODEBASE;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.redBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RedBlobVision.");
                vision.setColorBlobVisionEnabled(Vision.ColorBlobType.RedBlob, false);
            }

            if (vision.blueBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling BlueBlobVision.");
                vision.setColorBlobVisionEnabled(Vision.ColorBlobType.BlueBlob, false);
            }

            if (vision.limelightVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling LimelightVision.");
                vision.setLimelightVisionEnabled(0, false);
            }

            vision.close();
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method update all subsystem status on the dashboard.
     */
    public void updateStatus()
    {
        double currTime = TrcTimer.getCurrentTime();
        if (currTime > nextStatusUpdateTime)
        {
            int lineNum = 2;
            nextStatusUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
            if (robotDrive != null)
            {
                dashboard.displayPrintf(lineNum++, "DriveBase: Pose=%s", robotDrive.driveBase.getFieldPosition());
            }
            //
            // Display other subsystem status here.
            //
            if (RobotParams.Preferences.showSubsystems)
            {
                if (simpleMotor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "SimpleMotor: power=%.3f, pos=%.3f rev, vel=%.3f rpm",
                        simpleMotor.getPower(), simpleMotor.getPosition(), simpleMotor.getVelocity()*60.0);
                }

                if (simpleServo != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "SimpleServo: power=%.3f, pos=%.3f",
                        simpleServo.getPower(), simpleServo.getPosition());
                }

                if (elevator != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Elevator: power=%.3f, pos=%.3f/%.3f, limitSw=%s/%s",
                        elevator.getPower(), elevator.getPosition(), elevator.getPidTarget(),
                        elevator.isLowerLimitSwitchActive(), elevator.isUpperLimitSwitchActive());
                }

                if (arm != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Arm: power=%.3f, pos=%.3f/%.3f, limitSw=%s/%s",
                        arm.getPower(), arm.getPosition(), arm.getPidTarget(),
                        arm.isLowerLimitSwitchActive(), arm.isUpperLimitSwitchActive());
                }

                if (shooter != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Shooter1: power=%.3f, vel=%.3f, target=%.3f",
                        shooter.getShooterMotor1Power(), shooter.getShooterMotor1RPM(),
                        shooter.getShooterMotor1TargetRPM());
                    TrcMotor motor2 = shooter.getShooterMotor2();
                    if (motor2 != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Shooter2: power=%.3f, vel=%.3f, target=%.3f",
                            motor2.getPower(), shooter.getShooterMotor2RPM(),
                            shooter.getShooterMotor2TargetRPM());
                    }
                }

                if (intake != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Intake: power=%.3f, hasObject=%s, sensorState=%s",
                        intake.getPower(), intake.hasObject(), intake.getSensorState(intake.entryTrigger));
                }

                if (grabber != null)
                {
                    if (RobotParams.Grabber.USE_REV_2M_SENSOR)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Grabber: pos=%.3f, hasObject=%s, sensorValue=%.3f, autoActive=%s",
                            grabber.getPosition(), grabber.hasObject(), grabber.getSensorValue(),
                            grabber.isAutoAssistActive());
                    }
                    else if (RobotParams.Grabber.USE_DIGITAL_SENSOR)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Grabber: pos=%.3f, hasObject=%s, sensorState=%s, autoActive=%s",
                            grabber.getPosition(), grabber.hasObject(), grabber.getSensorState(),
                            grabber.isAutoAssistActive());
                    }
                }
            }
        }
    }   //updateStatus

    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");

        if (robotDrive != null)
        {
            // Cancel all auto-assist driving.
            robotDrive.cancel();
        }
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
    }   //setRobotStartPosition

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
