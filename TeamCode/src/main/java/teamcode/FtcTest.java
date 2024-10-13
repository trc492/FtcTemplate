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

import java.util.Arrays;
import java.util.Locale;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcGamepad;
import ftclib.driverio.FtcMenu;
import ftclib.driverio.FtcValueMenu;
import ftclib.robotcore.FtcPidCoeffCache;
import ftclib.vision.FtcLimelightVision;
import teamcode.vision.Vision;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcElapsedTimer;
import trclib.timer.TrcTimer;

/**
 * This class contains the Test Mode program. It extends FtcTeleOp so that we can teleop control the robot for
 * testing purposes. It provides numerous tests for diagnosing problems with the robot. It also provides tools
 * for tuning and calibration.
 */
@TeleOp(name="FtcTest", group="Ftcxxxx")
public class FtcTest extends FtcTeleOp
{
    private final String moduleName = getClass().getSimpleName();
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;

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
        PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        PURE_PURSUIT_DRIVE,
        CALIBRATE_SWERVE_STEERING
    }   //enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SENSORS_TEST;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;
        TrcPidController.PidCoefficients tunePidCoeffs = null;
        double tuneDistance = 0.0;
        double tuneHeading = 0.0;
        double tuneDrivePower = 0.0;

        @NonNull
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "test=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%1f " +
                "driveTime=%.1f " +
                "drivePower=%.1f " +
                "tunePidCoeffs=%s " +
                "tuneDistance=%.1f " +
                "tuneHeading=%.1f " +
                "tuneDrivePower=%.1f",
                test, xTarget, yTarget, turnTarget, driveTime, drivePower, tunePidCoeffs, tuneDistance, tuneHeading,
                tuneDrivePower);
        }   //toString

    }   //class TestChoices

    private final FtcPidCoeffCache pidCoeffCache = new FtcPidCoeffCache(RobotParams.Robot.TEAM_FOLDER_PATH);
    private final TestChoices testChoices = new TestChoices();
    private TrcElapsedTimer elapsedTimer = null;
    private FtcChoiceMenu<Test> testMenu = null;

    private TrcRobot.RobotCommand testCommand = null;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    // Swerve Steering Calibration.
    private boolean steerCalibrating = false;
    // Color Blob Vision Turning.
    private static final double[] COLOR_THRESHOLD_LOW_RANGES = {0.0, 0.0, 0.0};
    private static final double[] COLOR_THRESHOLD_HIGH_RANGES = {255.0, 255.0, 255.0};
    private double[] colorThresholds = null;
    private int colorThresholdIndex = 0;
    private double colorThresholdMultiplier = 1.0;
    private boolean teleOpControlEnabled = true;
    private long exposure;
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
        //
        // Create the robot command for the tests that need one.
        //
        switch (testChoices.test)
        {
            case DRIVE_MOTORS_TEST:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdDriveMotorsTest(robot.robotDrive.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, testChoices.drivePower, 0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, 0.0, testChoices.drivePower, 0.0);
                }
                break;

            case PID_DRIVE:
            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (robot.robotDrive != null &&
                    (testChoices.test != Test.TUNE_X_PID || robot.robotDrive.driveBase.supportsHolonomicDrive()))
                {
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                }
                break;
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
            case VISION_TEST:
                if (robot.vision != null)
                {
                    robot.vision.setCameraStreamEnabled(true);
                    if (robot.vision.vision != null)
                    {
                        exposure = robot.vision.vision.getCurrentExposure();
                    }
                    // Vision generally will impact performance, so we only enable it if it's needed.
                    if (robot.vision.aprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
                        robot.vision.setAprilTagVisionEnabled(true);
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

                    if (robot.vision.limelightVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling LimelightVision.");
                        robot.vision.setLimelightVisionEnabled(0, true);
                    }
                }
                break;

            case TUNE_COLORBLOB_VISION:
                if (robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "Enabling FtcRawEocvVision.");
                    robot.vision.setCameraStreamEnabled(true);
                    robot.vision.setRawColorBlobVisionEnabled(true);
                    colorThresholds = robot.vision.getRawColorBlobThresholds();
                    colorThresholdIndex = 0;
                    colorThresholdMultiplier = 1.0;
                    updateColorThresholds();
                }
                break;

            case PID_DRIVE:
                if (testCommand != null)
                {
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.drivePower, null,
                        new TrcPose2D(testChoices.xTarget*12.0, testChoices.yTarget*12.0, testChoices.turnTarget));
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case TUNE_X_PID:
                if (testCommand != null)
                {
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.drivePower, testChoices.tunePidCoeffs,
                        new TrcPose2D(testChoices.tuneDistance*12.0, 0.0, 0.0));
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case TUNE_Y_PID:
                if (testCommand != null)
                {
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.drivePower, testChoices.tunePidCoeffs,
                        new TrcPose2D(0.0, testChoices.tuneDistance*12.0, 0.0));
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case TUNE_TURN_PID:
                if (testCommand != null)
                {
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.drivePower, testChoices.tunePidCoeffs,
                        new TrcPose2D(0.0, 0.0, testChoices.tuneHeading));
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                    //
                    // Doing a 48x48-inch square box with robot heading always pointing to the center of the box.
                    //
                    // Set the current position as the absolute field origin so the path can be an absolute path.
                    TrcPose2D startPose = new TrcPose2D(0.0, 0.0, 0.0);
                    robot.robotDrive.driveBase.setFieldPosition(startPose);
                    robot.robotDrive.purePursuitDrive.start(startPose, false, new TrcPose2D(0.0, 48.0, 90.0));
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

        if (robot.robotDrive != null)
        {
            robot.robotDrive.cancel();
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
        int lineNum = 9;
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

                    if (prevTime != 0.0)
                    {
                        acceleration = (velocity - prevVelocity)/(currTime - prevTime);
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                }
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
                    doSensorsTest();
                    break;

                case VISION_TEST:
                case TUNE_COLORBLOB_VISION:
                    doVisionTest();
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotDrive != null)
                    {
                        robot.dashboard.displayPrintf(lineNum++, "Timed Drive: %.0f sec", testChoices.driveTime);
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

                case TUNE_X_PID:
                case TUNE_Y_PID:
                case TUNE_TURN_PID:
                    if (robot.robotDrive != null && testChoices.tunePidCoeffs != null)
                    {
                        robot.dashboard.displayPrintf(7, "TunePid=%s", testChoices.tunePidCoeffs);
                    }
                    //
                    // Intentionally falling through.
                    //
                case PURE_PURSUIT_DRIVE:
                case PID_DRIVE:
                    if (robot.robotDrive != null)
                    {
                        TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl;
                        if (testChoices.test == Test.PURE_PURSUIT_DRIVE)
                        {
                            xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                        }
                        else
                        {
                            xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s,rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                            robot.robotDrive.driveBase.getFieldPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_FRONT].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_LEFT_BACK].getPosition(),
                            robot.robotDrive.driveMotors[FtcRobotDrive.INDEX_RIGHT_BACK].getPosition());
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        yPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                        turnPidCtrl.displayPidInfo(lineNum);
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
            }
        }
    }   //periodic

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
                if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
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
                break;

            case B:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Increment to next color threshold index.
                        colorThresholdIndex++;
                        if (colorThresholdIndex >= colorThresholds.length)
                        {
                            colorThresholdIndex = colorThresholds.length - 1;
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VISION_TEST && pressed)
                {
                    fpsMeterEnabled = !fpsMeterEnabled;
                    robot.vision.setFpsMeterEnabled(fpsMeterEnabled);
                    robot.globalTracer.traceInfo(moduleName, "fpsMeterEnabled = %s", fpsMeterEnabled);
                }
                break;

            case X:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Decrement to previous color threshold index.
                        colorThresholdIndex--;
                        if (colorThresholdIndex < 0)
                        {
                            colorThresholdIndex = 0;
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case Y:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Set display to next intermediate Mat in the pipeline.
                        robot.vision.rawColorBlobVision.getPipeline().setNextVideoOutput();
                    }
                    passToTeleOp = false;
                }
                break;

            case LeftBumper:
                if (testChoices.test == Test.VISION_TEST && robot.vision != null && robot.vision.vision != null)
                {
                    if (pressed)
                    {
                        exposure -= 100;
                        robot.vision.vision.setManualExposure(exposure, null);
                    }
                    passToTeleOp = false;
                }
                break;

            case RightBumper:
                if (testChoices.test == Test.VISION_TEST && robot.vision != null && robot.vision.vision != null)
                {
                    if (pressed)
                    {
                        exposure += 100;
                        robot.vision.vision.setManualExposure(exposure, null);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadUp:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(0.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed &&
                        colorThresholds[colorThresholdIndex] + colorThresholdMultiplier <=
                        COLOR_THRESHOLD_HIGH_RANGES[colorThresholdIndex/2])
                    {
                        // Increment color threshold value.
                        colorThresholds[colorThresholdIndex] += colorThresholdMultiplier;
                        updateColorThresholds();
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(180.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed &&
                        colorThresholds[colorThresholdIndex] - colorThresholdMultiplier >=
                        COLOR_THRESHOLD_LOW_RANGES[colorThresholdIndex/2])
                    {
                        // Decrement color threshold value.
                        colorThresholds[colorThresholdIndex] -= colorThresholdMultiplier;
                        updateColorThresholds();
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(270.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed && colorThresholdMultiplier * 10.0 <= 100.0)
                    {
                        // Increment the significant multiplier.
                        colorThresholdMultiplier *= 10.0;
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadRight:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(90.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed && colorThresholdMultiplier / 10.0 >= 1.0)
                    {
                        // Decrement the significant multiplier.
                        colorThresholdMultiplier /= 10.0;
                    }
                    passToTeleOp = false;
                }
                break;

            case Back:
            case Start:
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
     * This method displays the current color thresholds on the dashboard.
     */
    private void updateColorThresholds()
    {
        robot.dashboard.displayPrintf(7, "Thresholds: %s", Arrays.toString(colorThresholds));
    }   //updateColorThresholds

    /**
     * This method creates and displays the test menus and record the selected choices.
     */
    private void doTestMenus()
    {
        //
        // Create menus.
        //
        testMenu = new FtcChoiceMenu<>("Tests:", null);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", testMenu, -180.0, 180.0, 5.0, 0.0, " %.0f deg");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", testMenu, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", testMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
        //
        // PID Tuning menus.
        //
        FtcValueMenu tuneKpMenu = new FtcValueMenu(
            "Kp:", testMenu, 0.0, 1.0, 0.001, this::getTuneKp, " %f");
        FtcValueMenu tuneKiMenu = new FtcValueMenu(
            "Ki:", tuneKpMenu, 0.0, 1.0, 0.001, this::getTuneKi, " %f");
        FtcValueMenu tuneKdMenu = new FtcValueMenu(
            "Kd:", tuneKiMenu, 0.0, 1.0, 0.001, this::getTuneKd, " %f");
        FtcValueMenu tuneKfMenu = new FtcValueMenu(
            "Kf:", tuneKdMenu, 0.0, 1.0, 0.001, this::getTuneKf, " %f");
        FtcValueMenu tuneDistanceMenu = new FtcValueMenu(
            "PID Tune distance:", tuneKfMenu, -10.0, 10.0, 0.5, 0.0,
            " %.1f ft");
        FtcValueMenu tuneHeadingMenu = new FtcValueMenu(
            "PID Tune heading:", tuneDistanceMenu, -180.0, 180.0, 5.0, 0.0,
            " %.0f deg");
        FtcValueMenu tuneDrivePowerMenu = new FtcValueMenu(
            "PID Tune drive power:", tuneHeadingMenu, -1.0, 1.0, 0.1, 1.0,
            " %.1f");
        //
        // Populate menus.
        //
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Tune ColorBlob vision", Test.TUNE_COLORBLOB_VISION, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false, xTargetMenu);
        testMenu.addChoice("Tune X PID", Test.TUNE_X_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID, false, tuneKpMenu);
        testMenu.addChoice("Pure Pursuit Drive", Test.PURE_PURSUIT_DRIVE, false);
        testMenu.addChoice("Calibrate Swerve Steering", Test.CALIBRATE_SWERVE_STEERING, false);

        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        tuneKpMenu.setChildMenu(tuneKiMenu);
        tuneKiMenu.setChildMenu(tuneKdMenu);
        tuneKdMenu.setChildMenu(tuneKfMenu);
        tuneKfMenu.setChildMenu(tuneDistanceMenu);
        tuneDistanceMenu.setChildMenu(tuneHeadingMenu);
        tuneHeadingMenu.setChildMenu(tuneDrivePowerMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu);
        //
        // Fetch choices.
        //
        testChoices.test = testMenu.getCurrentChoiceObject();
        testChoices.xTarget = xTargetMenu.getCurrentValue();
        testChoices.yTarget = yTargetMenu.getCurrentValue();
        testChoices.turnTarget = turnTargetMenu.getCurrentValue();
        testChoices.driveTime = driveTimeMenu.getCurrentValue();
        testChoices.drivePower = drivePowerMenu.getCurrentValue();
        testChoices.tunePidCoeffs = new TrcPidController.PidCoefficients(
            tuneKpMenu.getCurrentValue(), tuneKiMenu.getCurrentValue(),
            tuneKdMenu.getCurrentValue(),tuneKfMenu.getCurrentValue());
        testChoices.tuneDistance = tuneDistanceMenu.getCurrentValue();
        testChoices.tuneHeading = tuneHeadingMenu.getCurrentValue();
        testChoices.tuneDrivePower = tuneDrivePowerMenu.getCurrentValue();

        TrcPidController tunePidCtrl = getTunePidController(testChoices.test);
        if (tunePidCtrl != null)
        {
            //
            // Write the user input PID coefficients to a cache file so tune PID menu can read them as start value
            // next time.
            //
            pidCoeffCache.writeCachedPidCoeff(tunePidCtrl, testChoices.tunePidCoeffs);
        }
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Test Choices: %s", testChoices);
    }   //doTestMenus

    /**
     * This method returns the PID controller for the tune test.
     *
     * @param test specifies the selected test.
     * @return tune PID controller.
     */
    private TrcPidController getTunePidController(Test test)
    {
        TrcPidController pidCtrl;

        switch (test)
        {
            case TUNE_X_PID:
                pidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                break;

            case TUNE_Y_PID:
                pidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                break;

            case TUNE_TURN_PID:
                pidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                break;

            default:
                pidCtrl = null;
        }

        return pidCtrl;
    }   //getTunePidController

    /**
     * This method is called by the tuneKpMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kp value of the PID controller being tuned.
     */
    private double getTuneKp()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kP;
        }

        return value;
    }   //getTuneKp

    /**
     * This method is called by the tuneKiMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Ki value of the PID controller being tuned.
     */
    private double getTuneKi()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kI;
        }

        return value;
    }   //getTuneKi

    /**
     * This method is called by the tuneKdMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kd value of the PID controller being tuned.
     */
    private double getTuneKd()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kD;
        }

        return value;
    }   //getTuneKd

    /**
     * This method is called by the tuneKfMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kf value of the PID controller being tuned.
     */
    double getTuneKf()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kF;
        }

        return value;
    }   //getTuneKF

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        int lineNum = 9;
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
     */
    private void doVisionTest()
    {
        if (robot.vision != null)
        {
            int lineNum = 9;

            if (robot.vision.vision != null)
            {
                // displayExposureSettings is only available for VisionPortal.
                robot.vision.displayExposureSettings(lineNum++);
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
                robot.vision.getDetectedColorBlob(Vision.ColorBlobType.RedBlob, lineNum++);
            }

            if (robot.vision.blueBlobVision != null)
            {
                robot.vision.getDetectedColorBlob(Vision.ColorBlobType.BlueBlob, lineNum++);
            }

            if (robot.vision.limelightVision != null)
            {
                robot.vision.getLimelightDetectedObject(FtcLimelightVision.ResultType.Fiducial, null, lineNum++);
            }
        }
    }   //doVisionTest

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        return teleOpControlEnabled &&
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.DRIVE_SPEED_TEST);
    }   //allowTeleOp

}   //class FtcTest
