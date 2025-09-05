/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import ftclib.drivebase.FtcRobotDrive;
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcGamepad;
import ftclib.driverio.FtcMenu;
import ftclib.vision.FtcLimelightVision;
import teamcode.vision.Vision;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcElapsedTimer;
import trclib.timer.TrcTimer;

/**
 * This class contains the Test Mode program. It extends FtcTeleOp so that we can teleop control the robot for
 * testing purposes. It provides numerous tests for diagnosing problems with the robot. It also provides tools
 * for tuning and calibration.
 */
@TeleOp(name="FtcTest", group="FtcTeam")
public class FtcTest extends FtcTeleOp
{
    private final String moduleName = getClass().getSimpleName();
    private static final boolean logEvents = false;
    private static final boolean debugPid = false;

    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        VISION_TEST,
        TUNE_COLORBLOB_VISION,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_DRIVE_PID,
        TUNE_SUBSYSTEM,
        CALIBRATE_SWERVE_STEERING
    }   //enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SENSORS_TEST;

        @NonNull
        @Override
        public String toString()
        {
            return "test=\"" + test + "\"";
        }   //toString

    }   //class TestChoices

    private final TestChoices testChoices = new TestChoices();
    private TrcElapsedTimer elapsedTimer = null;

    private TrcRobot.RobotCommand testCommand = null;
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
    private boolean teleOpControlEnabled = true;
    private boolean fpsMeterEnabled = false;

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
        if (RobotParams.Preferences.useLoopPerformanceMonitor)
        {
            elapsedTimer = new TrcElapsedTimer("TestLoopMonitor", 2.0);
        }
        //
        // Test menus.
        //
        doTestMenus();
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
            case VISION_TEST:
                if (robot.vision != null)
                {
                    robot.vision.setCameraStreamEnabled(true);
                    // Vision generally will impact performance, so we only enable it if it's needed.
                    if (robot.vision.aprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Webcam.");
                        robot.vision.setAprilTagVisionEnabled(true);
                    }

                    if (robot.vision.limelightVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Limelight.");
                        robot.vision.setLimelightVisionEnabled(0, true);
                    }

                    if (robot.vision.redBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling RedBlobVision.");
                        robot.vision.setColorBlobVisionEnabled(Vision.ColorBlobType.RedBlob, true);
                    }

                    if (robot.vision.blueBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling BlueBlobVision.");
                        robot.vision.setColorBlobVisionEnabled(Vision.ColorBlobType.BlueBlob, true);
                    }
                }
                break;

            case TUNE_COLORBLOB_VISION:
                if (robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "Enabling FtcRawEocvVision.");
                    robot.vision.setCameraStreamEnabled(true);
                    robot.vision.setRawColorBlobVisionEnabled(true);
                    Dashboard.Vision.colorThresholds = robot.vision.getRawColorBlobThresholds();
                    Dashboard.Vision.filterContourParams = Vision.colorBlobFilterContourParams.clone();
                }
                break;

            case DRIVE_MOTORS_TEST:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotDrive.driveBase, robot.robotDrive.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, Dashboard.Drive.driveTime,
                        Dashboard.Drive.drivePower, 0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, Dashboard.Drive.driveTime,
                        0.0, Dashboard.Drive.drivePower, 0.0);
                }
                break;

            case PP_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(Dashboard.Drive.drivePower);
                    robot.robotDrive.purePursuitDrive.setRotOutputLimit(Dashboard.Drive.turnPower);
                    robot.robotDrive.purePursuitDrive.start(
                        true, robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxAcceleration,
                        new TrcPose2D(Dashboard.Drive.xTarget*12.0, Dashboard.Drive.yTarget*12.0,
                                      Dashboard.Drive.turnTarget));
                    robot.robotDrive.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.pidDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, Dashboard.Drive.drivePower, null,
                        new TrcPose2D(Dashboard.Drive.xTarget*12.0, Dashboard.Drive.yTarget*12.0,
                                      Dashboard.Drive.turnTarget));
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
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
                if (robot.robotDrive != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
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

            case TUNE_DRIVE_PID:
                if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                {
                    robot.dashboard.putObject(
                        "robotVelocity", robot.robotDrive.purePursuitDrive.getPathRobotVelocity());
                    robot.dashboard.putObject(
                        "targetVelocity", robot.robotDrive.purePursuitDrive.getPathTargetVelocity());
                    robot.dashboard.putObject(
                        "robotPosition", robot.robotDrive.purePursuitDrive.getPathRelativePosition());
                    robot.dashboard.putObject(
                        "targetPosition", robot.robotDrive.purePursuitDrive.getPathPositionTarget());
                }
                break;

            default:
                break;
        }

        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                15, "Period: %.3f(%.3f/%.3f)",
                elapsedTimer.getAverageElapsedTime(), elapsedTimer.getMinElapsedTime(),
                elapsedTimer.getMaxElapsedTime());
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            switch (testChoices.test)
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                    doSensorsTest(lineNum);
                    break;

                case VISION_TEST:
                case TUNE_COLORBLOB_VISION:
                    doVisionTest(lineNum);
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotDrive != null)
                    {
                        robot.dashboard.displayPrintf(lineNum++, "Timed Drive: %.0f sec", Dashboard.Drive.driveTime);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                        robot.dashboard.displayPrintf(
                            lineNum++, "rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_FRONT].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_BACK].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_BACK].getPosition());
                    }
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_DRIVE_PID:
                    if (robot.robotDrive != null)
                    {
                        TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                        if (testChoices.test == Test.PID_DRIVE && robot.robotDrive.pidDrive != null)
                        {
                            xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                        }
                        else if (robot.robotDrive.purePursuitDrive != null)
                        {
                            xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());

                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (yPidCtrl != null)
                        {
                            yPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (turnPidCtrl != null)
                        {
                            turnPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                    }
                    break;

                case CALIBRATE_SWERVE_STEERING:
                    if (robot.robotDrive != null && (robot.robotDrive instanceof FtcSwerveDrive) && steerCalibrating)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
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
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.TUNE_SUBSYSTEM  ||
                testChoices.test == Test.VISION_TEST || testChoices.test == Test.DRIVE_SPEED_TEST);
    }   //allowTeleOp

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
        robot.dashboard.displayPrintf(8, "Driver: %s=%s", button, pressed? "Pressed": "Released");
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
                    if (RobotParams.Preferences.tuneDriveBase)
                    {
                        // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                        teleOpControlEnabled = !pressed;
                        if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                        {
                            FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                            swerveDrive.setSteerAngle(0.0, false, true);
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Set display to next intermediate Mat in the pipeline.
                        robot.vision.rawColorBlobVision.getPipeline().setNextVideoOutput();
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VISION_TEST &&
                         robot.vision != null && robot.vision.isLimelightVisionEnabled())
                {
                    if (pressed)
                    {
                        int pipelineIndex = (robot.vision.limelightVision.getPipeline() + 1) %
                                            Vision.LimelightParams.NUM_PIPELINES;
                        robot.vision.limelightVision.setPipeline(pipelineIndex);
                        robot.globalTracer.traceInfo(moduleName, "Switch Limelight pipeline to " + pipelineIndex);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase)
                    {
                        // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                        teleOpControlEnabled = !pressed;
                        if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                        {
                            FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                            swerveDrive.setSteerAngle(180.0, false, true);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase)
                    {
                        // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                        teleOpControlEnabled = !pressed;
                        if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                        {
                            FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                            swerveDrive.setSteerAngle(270.0, false, true);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadRight:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    if (RobotParams.Preferences.tuneDriveBase)
                    {
                        // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                        teleOpControlEnabled = !pressed;
                        if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                        {
                            FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                            swerveDrive.setSteerAngle(90.0, false, true);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case Back:
                break;

            case Start:
                if (testChoices.test == Test.TUNE_DRIVE_PID)
                {
                    if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                robot.robotDrive.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotDrive.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        Dashboard.Drive.xTarget*12.0, Dashboard.Drive.yTarget*12.0,
                                        Dashboard.Drive.turnTarget));
                                tuneDriveAtEndPoint = false;
                            }
                            robot.robotDrive.purePursuitDrive.setXPositionPidCoefficients(Dashboard.Drive.xPidCoeffs);
                            robot.robotDrive.purePursuitDrive.setYPositionPidCoefficients(Dashboard.Drive.yPidCoeffs);
                            robot.robotDrive.purePursuitDrive.setTurnPidCoefficients(Dashboard.Drive.turnPidCoeffs);
                            robot.robotDrive.purePursuitDrive.setMoveOutputLimit(Dashboard.Drive.drivePower);
                            robot.robotDrive.purePursuitDrive.setRotOutputLimit(Dashboard.Drive.turnPower);
                            robot.robotDrive.purePursuitDrive.start(
                                false,
                                Dashboard.Drive.maxVelocity,
                                Dashboard.Drive.maxAcceleration,
                                Dashboard.Drive.maxDeceleration,
                                tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        passToTeleOp = false;
                    }
                }
                else if (testChoices.test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, "Tune subsystem: " + Dashboard.Subsystem.subsystemName);
                        if (Dashboard.Subsystem.subsystemName.isEmpty())
                        {
                            TrcSubsystem subsystem = TrcSubsystem.getSubsystem(Dashboard.Subsystem.subsystemName);
                            if (subsystem != null)
                            {
                                subsystem.prepSubsystemForTuning(Dashboard.Subsystem.tuneParams);
                            }
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                {
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;

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
                else if (testChoices.test == Test.VISION_TEST)
                {
                    if (pressed)
                    {
                        fpsMeterEnabled = !fpsMeterEnabled;
                        robot.vision.setFpsMeterEnabled(fpsMeterEnabled);
                        robot.globalTracer.traceInfo(moduleName, "fpsMeterEnabled = %s", fpsMeterEnabled);
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
        robot.dashboard.displayPrintf(8, "Operator: %s=%s", button, pressed? "Pressed": "Released");
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
            case DpadUp:
            case DpadDown:
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
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Tune ColorBlob vision", Test.TUNE_COLORBLOB_VISION, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false);
        testMenu.addChoice("Pure Pursuit Drive", Test.PP_DRIVE, false);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false);
        testMenu.addChoice("Tune Drive PID", Test.TUNE_DRIVE_PID, false);
        testMenu.addChoice("Tune Subsystem", Test.TUNE_SUBSYSTEM, false);
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
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     *
     * @param lineNum specifies the starting line number on the dashboard to display sensor states.
     */
    private void doSensorsTest(int lineNum)
    {
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        if (robot.robotDrive != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "DriveEnc: lf=%.0f,rf=%.0f,lb=%.0f,rb=%.0f",
                robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_BACK].getPosition(),
                robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_BACK].getPosition());

            if (robot.robotDrive instanceof FtcSwerveDrive)
            {
                FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                robot.dashboard.displayPrintf(
                    lineNum++, "SteerEnc: lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_LEFT_FRONT].getScaledPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_RIGHT_FRONT].getScaledPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_LEFT_BACK].getScaledPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_RIGHT_BACK].getScaledPosition());
                robot.dashboard.displayPrintf(
                    lineNum++, "SteerRaw: lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                    swerveDrive.steerEncoders[FtcRobotDrive.INDEX_RIGHT_BACK].getRawPosition());
            }

            if (robot.robotDrive.gyro != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "Gyro(x,y,z): Heading=(%.1f,%.1f,%.1f), Rate=(%.3f,%.3f,%.3f)",
                    robot.robotDrive.gyro.getXHeading().value, robot.robotDrive.gyro.getYHeading().value,
                    robot.robotDrive.gyro.getZHeading().value, robot.robotDrive.gyro.getXRotationRate().value,
                    robot.robotDrive.gyro.getYRotationRate().value, robot.robotDrive.gyro.getZRotationRate().value);
            }
        }
    }   //doSensorsTest

    /**
     * This method calls vision code to detect target objects and display their info.
     *
     * @param lineNum specifies the starting line number on the dashboard to display vision info.
     */
    private void doVisionTest(int lineNum)
    {
        if (robot.vision != null)
        {
            if (robot.vision.limelightVision != null)
            {
                robot.vision.getLimelightDetectedObject(
                    robot.vision.limelightVision.getPipeline() == 0?
                        FtcLimelightVision.ResultType.Fiducial: FtcLimelightVision.ResultType.Python,
                    null, lineNum++);
            }

            if (robot.vision.rawColorBlobVision != null)
            {
                robot.vision.getDetectedRawColorBlob(lineNum++);
            }

            if (robot.vision.aprilTagVision != null)
            {
                robot.vision.getDetectedAprilTag(null, lineNum++);
            }

            if (robot.vision.redBlobVision != null)
            {
                robot.vision.getDetectedColorBlob(Vision.ColorBlobType.RedBlob, 0.0, lineNum++);
            }

            if (robot.vision.blueBlobVision != null)
            {
                robot.vision.getDetectedColorBlob(Vision.ColorBlobType.BlueBlob, 0.0, lineNum++);
            }

            if (robot.vision.vision != null)
            {
                // displayExposureSettings is only available for VisionPortal.
                robot.vision.displayExposureSettings(lineNum++);
            }
        }
    }   //doVisionTest

}   //class FtcTest
