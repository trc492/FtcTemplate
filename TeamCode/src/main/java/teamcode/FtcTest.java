/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.drivebase.FtcRobotBase;
import ftclib.drivebase.FtcSwerveBase;
import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcGamepad;
import ftclib.driverio.FtcMenu;
import teamcode.vision.Vision;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class contains the Test Mode program. It extends FtcTeleOp so that we can teleop control the robot for
 * testing purposes. It provides numerous tests for diagnosing problems with the robot. It also provides tools
 * for tuning and calibration.
 */
@TeleOp(name="FtcTest", group="Ftc####")
public class FtcTest extends FtcTeleOp
{
    private final String moduleName = getClass().getSimpleName();
    private static final boolean logEvents = false;
    private static final boolean debugPid = false;

    private enum Test
    {
        SUBSYSTEMS_TEST,
        DRIVE_MOTORS_TEST,
        DRIVE_SPEED_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_PP_DRIVE,
        TUNE_PID_DRIVE,
        VISION_TEST,
        CALIBRATE_SWERVE_STEERING
    }   //enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SUBSYSTEMS_TEST;

        @NonNull
        @Override
        public String toString()
        {
            return "test=\"" + test + "\"";
        }   //toString

    }   //class TestChoices

    private final TestChoices testChoices = new TestChoices();
    private TrcRobot.RobotCommand testCommand = null;
    private boolean teleOpControlEnabled = true;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    // Tune Drive PID.
    private TrcPose2D tuneDriveStartPoint = null;
    private TrcPose2D tuneDriveEndPoint = null;
    private boolean tuneDriveAtEndPoint = false;
    // Swerve Steering Calibration.
    private boolean steerCalibrating = false;
    // Vision.
    private boolean fpsMeterEnabled = false;
    private Vision.ColorBlobType testVisionColorBlobType = Vision.ColorBlobType.Any;

    //
    // Overrides FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // TeleOp initialization.
        //
        super.robotInit();
        //
        // Test menus.
        //
        doTestMenus();
        // We are tuning subsystems, update Dashboard with the parameters from each subsystem.
        if (testChoices.test == Test.SUBSYSTEMS_TEST)
        {
            TrcSubsystem.updateSubsystemParamsToDashboard();
        }
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called before test mode is about to start so it can initialize appropriate subsystems for the
     * test.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        super.startMode(prevMode, nextMode);
        switch (testChoices.test)
        {
            case DRIVE_MOTORS_TEST:
                if (robot.robotBase != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotBase.driveBase, robot.robotBase.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotBase != null && robot.robotBase.driveBase.supportsHolonomicDrive())
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0,
                        Dashboard.Subsystem_Drivebase.driveBaseParams.driveTime,
                        Dashboard.Subsystem_Drivebase.driveBaseParams.xDrivePowerLimit, 0.0, 0.0);
                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotBase != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0,
                        Dashboard.Subsystem_Drivebase.driveBaseParams.driveTime,
                        0.0, Dashboard.Subsystem_Drivebase.driveBaseParams.yDrivePowerLimit, 0.0);
                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case PP_DRIVE:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(
                        Dashboard.Subsystem_Drivebase.driveBaseParams.yDrivePowerLimit);
                    robot.robotBase.purePursuitDrive.setRotOutputLimit(
                        Dashboard.Subsystem_Drivebase.driveBaseParams.turnPowerLimit);
                    robot.robotBase.purePursuitDrive.start(
                        true, Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveVelocity,
                        Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveAcceleration,
                        Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveDeceleration,
                        new TrcPose2D(Dashboard.Subsystem_Drivebase.driveBaseParams.xDriveTarget*12.0,
                                      Dashboard.Subsystem_Drivebase.driveBaseParams.yDriveTarget*12.0,
                                      Dashboard.Subsystem_Drivebase.driveBaseParams.turnTarget));
                    robot.robotBase.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case PID_DRIVE:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    testCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    robot.robotBase.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case VISION_TEST:
                if (robot.vision != null)
                {
                    if (robot.vision.webcamAprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Webcam.");
                        robot.vision.setWebcamAprilTagVisionEnabled(true);
                    }

                    if (robot.vision.limelightVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Limelight.");
                        robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
                    }

                    if (robot.vision.colorBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling ColorBlobVision.");
                        robot.vision.setColorBlobVisionEnabled(Vision.ColorBlobType.Any, true);
                    }
                }
                break;

            case DRIVE_SPEED_TEST:
            case TUNE_PP_DRIVE:
            case TUNE_PID_DRIVE:
                robot.dashboard.disableDashboardUpdate();
                break;
        }
    }   //startMode

    /**
     * This method is called before test mode is about to exit so it can do appropriate cleanup.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (testCommand != null)
        {
            testCommand.cancel();
        }

        super.stopMode(prevMode, nextMode);
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
        int lineNum = 1;
        //
        // Run the testCommand if any.
        //
        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Display test status.
        //
        switch (testChoices.test)
        {
            case DRIVE_SPEED_TEST:
                if (robot.robotBase != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotBase.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;
                    double deceleration = 0.0;
                    double deltaTime = currTime - prevTime;

                    if (prevTime != 0.0)
                    {
                        if (velocity > prevVelocity)
                        {
                            acceleration = (velocity - prevVelocity)/deltaTime;
                        }
                        else
                        {
                            deceleration = (prevVelocity - velocity)/deltaTime;
                        }
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    if (deceleration > maxDriveDeceleration)
                    {
                        maxDriveDeceleration = deceleration;
                    }

                    if (velPose.angle > maxTurnVelocity)
                    {
                        maxTurnVelocity = velPose.angle;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                }
                break;

            case TUNE_PP_DRIVE:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.dashboard.putNumber(
                        "robotVelocity", robot.robotBase.purePursuitDrive.getPathRobotVelocity());
                    robot.dashboard.putNumber(
                        "targetVelocity", robot.robotBase.purePursuitDrive.getPathTargetVelocity());
                    robot.dashboard.putNumber(
                        "robotPosition", robot.robotBase.purePursuitDrive.getPathRelativePosition());
                    robot.dashboard.putNumber(
                        "targetPosition", robot.robotBase.purePursuitDrive.getPathPositionTarget());
                }
                break;

            default:
                break;
        }
        //
        // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
        //
        allowAnalogControl = allowTeleOp();
        super.periodic(elapsedTime, slowPeriodicLoop);

        if (slowPeriodicLoop)
        {
            switch (testChoices.test)
            {
                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotBase != null)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "Timed Drive: %.0f sec", Dashboard.Subsystem_Drivebase.driveBaseParams.driveTime);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                        robot.dashboard.displayPrintf(
                            lineNum++, "rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                            robot.robotBase.driveMotors[FtcRobotBase.INDEX_FRONT_LEFT].getPosition(),
                            robot.robotBase.driveMotors[FtcRobotBase.INDEX_FRONT_RIGHT].getPosition(),
                            robot.robotBase.driveMotors[FtcRobotBase.INDEX_BACK_LEFT].getPosition(),
                            robot.robotBase.driveMotors[FtcRobotBase.INDEX_BACK_RIGHT].getPosition());
                    }
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_PP_DRIVE:
                case TUNE_PID_DRIVE:
                    if (robot.robotBase != null)
                    {
                        TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                        if ((testChoices.test == Test.PID_DRIVE || testChoices.test == Test.TUNE_PID_DRIVE) &&
                            robot.robotBase.pidDrive != null)
                        {
                            xPidCtrl = robot.robotBase.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotBase.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotBase.pidDrive.getTurnPidCtrl();
                        }
                        else if ((testChoices.test == Test.PP_DRIVE || testChoices.test == Test.TUNE_PP_DRIVE) &&
                                 robot.robotBase.purePursuitDrive != null)
                        {
                            xPidCtrl = robot.robotBase.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotBase.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotBase.purePursuitDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());

                        if (xPidCtrl != null)
                        {
                            robot.dashboard.putNumber("xPidPos", xPidCtrl.getCurrentInput());
                            robot.dashboard.putNumber("xPidTarget", xPidCtrl.getPositionSetpoint());
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (yPidCtrl != null)
                        {
                            robot.dashboard.putNumber("yPidPos", yPidCtrl.getCurrentInput());
                            robot.dashboard.putNumber("yPidTarget", yPidCtrl.getPositionSetpoint());
                            yPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (turnPidCtrl != null)
                        {
                            robot.dashboard.putNumber("turnPidPos", turnPidCtrl.getCurrentInput());
                            robot.dashboard.putNumber("turnPidTarget", turnPidCtrl.getPositionSetpoint());
                            turnPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                    }
                    break;

                case VISION_TEST:
                    lineNum = doVisionTest(lineNum);
                    break;

                case CALIBRATE_SWERVE_STEERING:
                    if (robot.robotBase != null && (robot.robotBase instanceof FtcSwerveBase) && steerCalibrating)
                    {
                        FtcSwerveBase swerveDrive = (FtcSwerveBase) robot.robotBase;
                        swerveDrive.runSteeringCalibration();
                        swerveDrive.displaySteerZeroCalibration(lineNum);
                    }
                    break;

                default:
                    break;
            }
        }
    }   //periodic

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        return teleOpControlEnabled &&
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.VISION_TEST ||
                testChoices.test == Test.DRIVE_SPEED_TEST);
    }   //allowTeleOp

    /**
     * This method tunes the drive motors velocity control as well as steering PID if it's a Swerve Drive Base.
     *
     * @param velocity specifies the velocity to be set to all drive motors.
     * @param steerAngle specifies the steer angle if it is swerve drive. For other drive bases, steer angle 0 and 90
     *        will turn drive motors forward, 180 and 270 will turn them backward. This allows the user to run the
     *        robot back and forth for tuning drive motor velocity control PID. It also allows the user to tune
     *        steer motor PID.
     */
    private void tuneDriveMotors(double velocity, double steerAngle)
    {
        if (robot.robotBase instanceof FtcSwerveBase)
        {
            FtcSwerveBase swerveDrive = (FtcSwerveBase) robot.robotBase;
            swerveDrive.setSteerAngle(steerAngle, false, true);
        }
        else if (steerAngle == 180.0 || steerAngle == 270.0)
        {
            velocity = -velocity;
        }

        if (Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorVelControlEnabled &&
            Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorVelPidCoeffs != null)
        {
            // DriveMotor velocity control is enabled, let's tune DriveMotor velocity PID.
            for (TrcMotor motor: robot.robotBase.driveMotors)
            {
                motor.setVelocity(velocity);
            }
        }
    }   //tuneDriveMotors

    //
    // Overrides TrcGameController.ButtonHandler in TeleOp.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void driverButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        robot.dashboard.displayPrintf(15, "Driver: %s=%s", button, pressed? "Pressed": "Released");
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
                break;

            case DpadUp:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            teleOpControlEnabled = false;
                            tuneDriveMotors(Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorMaxVelocity, 0.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            teleOpControlEnabled = true;
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VISION_TEST && robot.vision != null)
                {
                    if (pressed)
                    {
                        if (robot.vision.colorBlobVision != null)
                        {
                            // Set display to next intermediate Mat in the pipeline.
                            if (robot.vision.isColorBlobVisionEnabled(Vision.ColorBlobType.Any))
                            {
                                robot.vision.colorBlobVision.getVisionProcessor().getPipeline().setNextVideoOutput();
                            }
                        }
                        else if (robot.vision.isLimelightVisionEnabled())
                        {
                            int pipelineIndex = (robot.vision.limelightVision.getPipeline() + 1) %
                                                Vision.NUM_LIMELIGHT_PIPELINES;
                            robot.vision.limelightVision.setPipeline(pipelineIndex);
                            robot.globalTracer.traceInfo(moduleName, "Switch Limelight pipeline to " + pipelineIndex);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            teleOpControlEnabled = false;
                            tuneDriveMotors(Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorMaxVelocity, 180.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            teleOpControlEnabled = true;
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            teleOpControlEnabled = false;
                            tuneDriveMotors(Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorMaxVelocity, 270.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            teleOpControlEnabled = true;
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadRight:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            teleOpControlEnabled = false;
                            tuneDriveMotors(Dashboard.Subsystem_Drivebase.driveBaseParams.driveMotorMaxVelocity, 90.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            teleOpControlEnabled = true;
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VISION_TEST && robot.vision != null &&
                         robot.vision.colorBlobVision != null)
                {
                    if (pressed)
                    {
                        if (testVisionColorBlobType == Vision.ColorBlobType.Any)
                        {
                            testVisionColorBlobType = Vision.ColorBlobType.RedBlob;
                        }
                        else if (testVisionColorBlobType == Vision.ColorBlobType.RedBlob)
                        {
                            testVisionColorBlobType = Vision.ColorBlobType.BlueBlob;
                        }
                        else
                        {
                            testVisionColorBlobType = Vision.ColorBlobType.Any;
                        }

                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Switch ColorBlob Vision to %s", testVisionColorBlobType);
                        robot.vision.setColorBlobVisionEnabled(testVisionColorBlobType, true);
                    }
                    passToTeleOp = false;
                }
                break;

            case Back:
                break;

            case Start:
                if (testChoices.test == Test.TUNE_PP_DRIVE || testChoices.test == Test.TUNE_PID_DRIVE)
                {
                    if (robot.robotBase != null &&
                        (robot.robotBase.purePursuitDrive != null || robot.robotBase.pidDrive != null))
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                // At starting point.
                                robot.robotBase.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotBase.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        Dashboard.Subsystem_Drivebase.driveBaseParams.xDriveTarget*12.0,
                                        Dashboard.Subsystem_Drivebase.driveBaseParams.yDriveTarget*12.0,
                                        Dashboard.Subsystem_Drivebase.driveBaseParams.turnTarget));
                            }

                            if (testChoices.test == Test.TUNE_PP_DRIVE && robot.robotBase.purePursuitDrive != null)
                            {
                                robot.robotBase.purePursuitDrive.start(
                                    false,
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveVelocity,
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveAcceleration,
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.profiledMaxDriveDeceleration,
                                    tuneDriveAtEndPoint ? tuneDriveStartPoint : tuneDriveEndPoint);
                            }
                            else if (robot.robotBase.pidDrive != null)
                            {
                                robot.robotBase.pidDrive.setAbsoluteTarget(
                                    tuneDriveAtEndPoint ? tuneDriveStartPoint : tuneDriveEndPoint, true);
                                TrcDbgTrace.globalTraceInfo(
                                    moduleName, "PidDrivePose=" + robot.robotBase.pidDrive.getAbsoluteTargetPose());
                            }
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        else
                        {
                            robot.robotBase.cancel();
                        }
                        passToTeleOp = false;
                    }
                }
                else if (testChoices.test == Test.PID_DRIVE)
                {
                    if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                    {
                        if (pressed)
                        {
                            robot.robotBase.driveBase.resetOdometry();
                            ((CmdPidDrive) testCommand).start(
                                0.0, Dashboard.Subsystem_Drivebase.driveBaseParams.yDrivePowerLimit, null,
                                new TrcPose2D(
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.xDriveTarget*12.0,
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.yDriveTarget*12.0,
                                    Dashboard.Subsystem_Drivebase.driveBaseParams.turnTarget));
                        }
                    }
                }
                else if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (pressed)
                    {
                        TrcSubsystem.updateSubsystemParamsFromDashboard();
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VISION_TEST && robot.vision != null)
                {
                    if (pressed)
                    {
                        fpsMeterEnabled = !fpsMeterEnabled;
                        robot.vision.setFpsMeterEnabled(fpsMeterEnabled);
                        robot.globalTracer.traceInfo(moduleName, "fpsMeterEnabled = %s", fpsMeterEnabled);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                {
                    if (pressed && robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase)
                    {
                        FtcSwerveBase swerveDrive = (FtcSwerveBase) robot.robotBase;

                        steerCalibrating = !steerCalibrating;
                        if (steerCalibrating)
                        {
                            // Start steer calibration.
                            swerveDrive.startSteeringCalibration();
                        }
                        else
                        {
                            // Stop steer calibration.
                            swerveDrive.stopSteeringCalibration();
                        }
                    }
                    passToTeleOp = false;
                }
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.driverButtonEvent(button, pressed);
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void operatorButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        robot.dashboard.displayPrintf(15, "Operator: %s=%s", button, pressed? "Pressed": "Released");
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
                break;

            case DpadUp:
                if (testChoices.test == Test.SUBSYSTEMS_TEST && Dashboard.DashboardParams.tuneSubsystemName != null)
                {
                    if (pressed)
                    {
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (testChoices.test == Test.SUBSYSTEMS_TEST && Dashboard.DashboardParams.tuneSubsystemName != null)
                {
                    if (pressed)
                    {
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
            case DpadRight:
            case Back:
            case Start:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorButtonEvent(button, pressed);
        }
    }   //operatorButtonEvent

    /**
     * This method creates and displays the test menus and record the selected choices.
     */
    private void doTestMenus()
    {
        //
        // Create menus.
        //
        FtcChoiceMenu<Test> testMenu = new FtcChoiceMenu<>("Tests:", null);
        //
        // Populate menus.
        //
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false);
        testMenu.addChoice("Pure Pursuit Drive", Test.PP_DRIVE, false);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false);
        testMenu.addChoice("Tune PurePursuit Drive", Test.TUNE_PP_DRIVE, false);
        testMenu.addChoice("Tune PID Drive", Test.TUNE_PID_DRIVE, false);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Calibrate Swerve Steering", Test.CALIBRATE_SWERVE_STEERING, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu);
        //
        // Fetch choices.
        //
        testChoices.test = testMenu.getCurrentChoiceObject();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Test Choices: %s", testChoices);
    }   //doTestMenus

    /**
     * This method calls vision code to detect target objects and display their info.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     */
    private int doVisionTest(int lineNum)
    {
        if (robot.vision != null)
        {
            lineNum = robot.vision.updateStatus(lineNum, true);
        }

        return lineNum;
    }   //doVisionTest

}   //class FtcTest
