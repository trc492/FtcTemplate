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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import trclib.drivebase.TrcDriveBase;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftcxxxx")
public class FtcTeleOp extends FtcOpMode
{
    private static final String moduleName = FtcTeleOp.class.getSimpleName();

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale;
    private double turnPowerScale;
    private boolean driverAltFunc = false;
    private boolean operatorAltFunc = false;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        drivePowerScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
        turnPowerScale = RobotParams.Robot.TURN_NORMAL_SCALE;

        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Unknown_TeleOp";
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setButtonEventHandler(this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        driverGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);
        setDriveOrientation(RobotParams.Robot.DRIVE_ORIENTATION);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        //
        // Enable AprilTag vision for re-localization.
        //
        if (robot.vision != null && robot.vision.aprilTagVision != null)
        {
            robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
            robot.vision.setAprilTagVisionEnabled(true);
        }
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        printPerformanceMetrics();
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            if (robot.robotDrive != null)
            {
                // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                if (relocalizing)
                {
                    if (robotFieldPose == null)
                    {
                        robotFieldPose = robot.vision.getRobotFieldPose();
                    }
                }
                else
                {
                    double[] inputs = driverGamepad.getDriveInputs(
                        RobotParams.Robot.DRIVE_MODE, true, drivePowerScale, turnPowerScale);

                    if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(
                            null, inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveGyroAngle());
                    }
                    else
                    {
                        robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                    }
                    robot.dashboard.displayPrintf(
                        1, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                        inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveOrientation());
                }
            }
            //
            // Other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
            }
            // Display subsystem status.
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotDrive != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=" + orientation);
            robot.robotDrive.driveBase.setDriveOrientation(
                orientation, orientation == TrcDriveBase.DriveOrientation.FIELD);
            if (robot.blinkin != null)
            {
                robot.blinkin.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Driver: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (driverAltFunc)
                {
                    if (pressed && robot.robotDrive != null)
                    {
                        if (robot.robotDrive.driveBase.isGyroAssistEnabled())
                        {
                            // Disable GyroAssist drive.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                        }
                        else
                        {
                            // Enable GyroAssist drive.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                        }
                    }
                }
                else
                {
                    if (pressed && robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        if (robot.robotDrive.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD)
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling FIELD mode.");
                            setDriveOrientation(TrcDriveBase.DriveOrientation.FIELD);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling ROBOT mode.");
                            setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
                        }
                    }
                }
                break;

            case B:
            case X:
            case Y:
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + pressed);
                driverAltFunc = pressed;
                break;

            case RightBumper:
                // Press and hold for slow drive.
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
                    drivePowerScale = RobotParams.Robot.DRIVE_SLOW_SCALE;
                    turnPowerScale = RobotParams.Robot.TURN_SLOW_SCALE;
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
                    drivePowerScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
                    turnPowerScale = RobotParams.Robot.TURN_NORMAL_SCALE;
                }
                break;

            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate();
                    if (robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        // Drive base is a Swerve Drive, align all steering wheels forward.
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set SteerAngle to zero.");
                        ((FtcSwerveDrive) robot.robotDrive).setSteerAngle(0.0, false, false);
                    }
                }
                break;

            case Start:
                if (robot.vision != null && robot.vision.aprilTagVision != null && robot.robotDrive != null)
                {
                    // On press of the button, we will start looking for AprilTag for re-localization.
                    // On release of the button, we will set the robot's field location if we found the AprilTag.
                    relocalizing = pressed;
                    if (!pressed)
                    {
                        if (robotFieldPose != null)
                        {
                            // Vision found an AprilTag, set the new robot field location.
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Finish re-localizing: pose=" + robotFieldPose);
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            robotFieldPose = null;
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                    }
                }
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Operator: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + pressed);
                operatorAltFunc = pressed;
                break;

            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    // Zero calibrate all subsystems (arm, elevator and turret).
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate(moduleName);
                }
                break;

            case Start:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
