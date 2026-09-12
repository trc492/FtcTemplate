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

import ftclib.drivebase.FtcSwerveBase;
import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcGamepad;
import ftclib.driverio.FtcMenu;
import ftclib.driverio.FtcValueMenu;
import teamcode.vision.Vision;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.motor.TrcMotor;
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
@TeleOp(name="FtcTest", group="Ftc####")
public class FtcTest extends FtcTeleOp
{
    private final String moduleName = getClass().getSimpleName();
    private static final boolean logEvents = false;
    private static final boolean debugPid = false;
    //
    // Tests.
    //
    private enum Test
    {
        SubsystemsTest,
        DriveSpeedTest,
        DriveMotorsTest,
        XTimedDrive,
        YTimedDrive,
        PurePursuitDrive,
        PidDrive,
        TuneDriveBasePid,
        TuneSubsystem,
        VisionTest,
        SwerveCalibration
    }   //enum Test

    /**
     * This class encapsulates all test choices for test mode.
     */
    private static class TestChoices
    {
        // Standard choice menus.
        private final FtcChoiceMenu<Test> testMenu;
        private final FtcValueMenu xTargetMenu;
        private final FtcValueMenu yTargetMenu;
        private final FtcValueMenu turnTargetMenu;
        private final FtcValueMenu drivePowerMenu;
        private final FtcValueMenu turnPowerMenu;
        private final FtcValueMenu timedDrivePowerMenu;
        private final FtcValueMenu timedDriveTimeMenu;

        private Test test = Test.SubsystemsTest;
        private double driveXTarget = 0.0;
        private double driveYTarget = 0.0;
        private double turnTarget = 0.0;
        private double drivePower = 0.0;
        private double turnPower = 0.0;
        private double timedDrivePower = 0.0;
        private double timedDriveTime = 0.0;

        public TestChoices()
        {
            //
            // Construct menus.
            //
            testMenu = new FtcChoiceMenu<>("Tests:", null);
            xTargetMenu = new FtcValueMenu("xTarget:", testMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
            yTargetMenu = new FtcValueMenu("yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
            turnTargetMenu = new FtcValueMenu("turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f deg");
            drivePowerMenu = new FtcValueMenu("Drive power:", turnTargetMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            turnPowerMenu = new FtcValueMenu("Turn power:", drivePowerMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            timedDrivePowerMenu = new FtcValueMenu("Drive power:", testMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            timedDriveTimeMenu = new FtcValueMenu("Drive time:", timedDrivePowerMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
            //
            // Populate menus.
            //
            testMenu.addChoice("Subsystems test", Test.SubsystemsTest, true);
            testMenu.addChoice("Drive speed test", Test.DriveSpeedTest, false);
            testMenu.addChoice("Drive motors test", Test.DriveMotorsTest, false);
            testMenu.addChoice("X Timed drive", Test.XTimedDrive, false, timedDrivePowerMenu);
            testMenu.addChoice("Y Timed drive", Test.YTimedDrive, false, timedDrivePowerMenu);
            testMenu.addChoice("Pure Pursuit Drive", Test.PurePursuitDrive, false, xTargetMenu);
            testMenu.addChoice("PID drive", Test.PidDrive, false, xTargetMenu);
            testMenu.addChoice("Tune DriveBase PID", Test.TuneDriveBasePid, false);
            testMenu.addChoice("Tune Subsystem", Test.TuneSubsystem, false);
            testMenu.addChoice("Vision test", Test.VisionTest, false);
            testMenu.addChoice("Calibrate Swerve Steering", Test.SwerveCalibration, false);
            //
            // Link Value Menus to their children.
            //
            xTargetMenu.setChildMenu(yTargetMenu);
            yTargetMenu.setChildMenu(turnTargetMenu);
            turnTargetMenu.setChildMenu(drivePowerMenu);
            drivePowerMenu.setChildMenu(turnPowerMenu);
            timedDrivePowerMenu.setChildMenu(timedDriveTimeMenu);
        }   //TestChoices

        /**
         * This method displays the Auto Choice menus for selection and stores the choices.
         */
        private void fetchChoices(FtcDashboard dashboard)
        {
            //
            // Traverse menus.
            //
            FtcMenu.walkMenuTree(testMenu);
            //
            // Fetch choices.
            //
            test = testMenu.getCurrentChoiceObject();
            driveXTarget = xTargetMenu.getCurrentValue();
            driveYTarget = yTargetMenu.getCurrentValue();
            turnTarget = turnTargetMenu.getCurrentValue();
            drivePower = drivePowerMenu.getCurrentValue();
            turnPower = turnPowerMenu.getCurrentValue();
            timedDrivePower = timedDrivePowerMenu.getCurrentValue();
            timedDriveTime = timedDriveTimeMenu.getCurrentValue();
            //
            // Show choices.
            //
            if (dashboard != null)
            {
                dashboard.displayPrintf(1, "Test Choices: %s", testChoices);
            }
        }   //fetchChoices

        @NonNull
        @Override
        public String toString()
        {
            return "test=" + test + "\" " +
                   "xDistance=" + driveXTarget + " ft " +
                   "yDistance=" + driveYTarget + " ft " +
                   "turnDegrees=" + turnTarget + " deg " +
                   "drivePower=" + drivePower + "\" " +
                   "turnPower=" + turnPower + "\" " +
                   "timedDrivePower=" + timedDrivePower + "\" " +
                   "timedDriveTime=" + timedDriveTime + " sec ";
        }   //toString
    }   //class TestChoices

    private static final TestChoices testChoices = new TestChoices();
    private static final TrcElapsedTimer loopPerfTimer =
        RobotParams.Preferences.useLoopPerformanceMonitor? new TrcElapsedTimer("loopPerfMonitor", 2.0): null;
    private TrcRobot.RobotCommand testCommand = null;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private Double prevTime = null;
    private Double prevVelocity = null;
    // Tune Drive PID.
    private TrcPose2D tuneDriveStartPoint = null;
    private TrcPose2D tuneDriveEndPoint = null;
    private boolean tuneDriveAtEndPoint = false;
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
        testChoices.fetchChoices(robot.dashboard);
    }   //robotInit

    //
    // Extending TrcRobot.RunMode methods in FtcTeleOp.
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
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);

        switch (testChoices.test)
        {
            case DriveSpeedTest:
                maxDriveVelocity = 0.0;
                maxDriveAcceleration = 0.0;
                maxDriveDeceleration = 0.0;
                maxTurnVelocity = 0.0;
                prevTime = null;
                prevVelocity = null;
                break;

            case DriveMotorsTest:
                if (robot.robotBase != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotBase.driveBase, robot.robotBase.driveMotors, 5.0, 0.5);
                    testCommand.start();
                }
                break;

            case XTimedDrive:
            case YTimedDrive:
                if (robot.robotBase != null &&
                    (testChoices.test == Test.YTimedDrive || robot.robotBase.driveBase.supportsHolonomicDrive()))
                {
                    double xPower, yPower;

                    xPower = yPower = testChoices.drivePower;
                    if (testChoices.test == Test.XTimedDrive)
                    {
                        yPower = 0.0;
                    }
                    else
                    {
                        xPower = 0.0;
                    }
                    robot.robotBase.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0, testChoices.timedDriveTime, xPower, yPower, 0.0);
                    testCommand.start();
                }
                break;

            case PurePursuitDrive:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPurePursuitDrive(
                        robot.robotBase.driveBase,
                        Dashboard.SubsystemDrivebase.driveBaseParams.xDrivePidCoeffs,
                        Dashboard.SubsystemDrivebase.driveBaseParams.yDrivePidCoeffs,
                        Dashboard.SubsystemDrivebase.driveBaseParams.turnPidCoeffs,
                        Dashboard.SubsystemDrivebase.driveBaseParams.velPidCoeffs);

                    ((CmdPurePursuitDrive) testCommand).startPath(
                        0.0, true,
                        Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveVelocity,
                        Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveAcceleration,
                        Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveDeceleration,
                        testChoices.drivePower, testChoices.turnPower,
                        new TrcPose2D(
                            testChoices.driveXTarget*12.0,
                            testChoices.driveYTarget*12.0,
                            testChoices.turnTarget));
                    robot.robotBase.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
//                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case PidDrive:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);

                    ((CmdPidDrive) testCommand).startPath(
                        0.0, testChoices.drivePower, testChoices.turnPower, null,
                        new TrcPose2D(
                            testChoices.driveXTarget*12.0,
                            testChoices.driveYTarget*12.0,
                            testChoices.turnTarget));
                    robot.robotBase.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
//                    robot.dashboard.disableDashboardUpdate();
                }
                break;

            case VisionTest:
                if (robot.vision != null)
                {
                    if (robot.vision.frontCamAprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Webcam.");
                        robot.vision.setWebcamAprilTagVisionEnabled(true);
                    }

                    if (robot.vision.limelightVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision for Limelight.");
                        robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.AprilTag, true);
                    }

                    if (robot.vision.backCamColorBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling ColorBlobVision.");
                        robot.vision.setColorBlobVisionEnabled(Vision.ColorBlobType.Any, true);
                    }
                }
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Start Swerve Calibration.");
                    setControlsEnabled(false);
                    ((FtcSwerveBase) robot.robotBase).startSteeringCalibration();
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
        switch (testChoices.test)
        {
            case XTimedDrive:
            case YTimedDrive:
                // Cancel GyroAssist in case we turned it on for timed drive.
                robot.robotBase.driveBase.setGyroAssistEnabled(null);
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Stop Swerve Calibration.");
                    ((FtcSwerveBase) robot.robotBase).stopSteeringCalibration();
                }
                break;

            default:
                break;
        }

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
            case DriveSpeedTest:
                if (robot.robotBase != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotBase.driveBase.getRobotVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;
                    double deceleration = 0.0;
                    Double deltaTime = prevTime == null? null: currTime - prevTime;

                    if (deltaTime != null)
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

                    if (slowPeriodicLoop)
                    {
                        robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                    }
                }
                break;

            case XTimedDrive:
            case YTimedDrive:
                if (slowPeriodicLoop && robot.robotBase != null)
                {
                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                    robot.dashboard.displayPrintf(
                        lineNum++, "rawEnc=fl:%.0f,fr:%.0f,bl:%.0f,br:%.0f",
                        robot.robotBase.driveMotors[MotorIndex.FrontLeft.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.BackLeft.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.BackRight.value].getPosition());
                }
                break;

            case TuneDriveBasePid:
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
                // Intentionally falling through.
            case PurePursuitDrive:
            case PidDrive:
                if (robot.robotBase != null && slowPeriodicLoop)
                {
                    TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                    if ((testChoices.test == Test.PurePursuitDrive || testChoices.test == Test.TuneDriveBasePid) &&
                        robot.robotBase.purePursuitDrive != null)
                    {
                        xPidCtrl = robot.robotBase.purePursuitDrive.getXPosPidCtrl();
                        yPidCtrl = robot.robotBase.purePursuitDrive.getYPosPidCtrl();
                        turnPidCtrl = robot.robotBase.purePursuitDrive.getTurnPidCtrl();
                    }
                    else if (testChoices.test == Test.PidDrive && robot.robotBase.pidDrive != null)
                    {
                        xPidCtrl = robot.robotBase.pidDrive.getXPidCtrl();
                        yPidCtrl = robot.robotBase.pidDrive.getYPidCtrl();
                        turnPidCtrl = robot.robotBase.pidDrive.getTurnPidCtrl();
                    }

                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
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

            case VisionTest:
                if (robot.vision != null && slowPeriodicLoop)
                {
                    lineNum = robot.vision.updateStatus(lineNum, true);
                }
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase && slowPeriodicLoop)
                {
                    FtcSwerveBase swerveBase = (FtcSwerveBase) robot.robotBase;
                    swerveBase.runSteeringCalibration();
                    swerveBase.displaySteerZeroCalibration(lineNum);
                }
                break;

            default:
                break;
        }
        //
        // Call super.runPeriodic only if you need TeleOp control of the robot for some tests.
        //
        if (testChoices.test == Test.SubsystemsTest || testChoices.test == Test.TuneSubsystem ||
            testChoices.test == Test.VisionTest || testChoices.test == Test.DriveSpeedTest)
        {
            super.periodic(elapsedTime, true);
        }

        if (loopPerfTimer != null)
        {
            loopPerfTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                14, "Period: %.3f(%.3f/%.3f)",
                loopPerfTimer.getAverageElapsedTime(), loopPerfTimer.getMinElapsedTime(),
                loopPerfTimer.getMaxElapsedTime());
        }
    }   //periodic

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

        if (Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorVelControlEnabled &&
            Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorVelPidCoeffs != null)
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
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Driver: " + button + "=" + (pressed? "pressed": "released"));
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
                if (testChoices.test == Test.SubsystemsTest)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            setControlsEnabled(false);
                            tuneDriveMotors(Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorMaxVelocity, 0.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            setControlsEnabled(true);
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        TrcSubsystem.setSubsystemTuneTargetUp(Dashboard.TuneSubsystem.subsystemName);
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> SetTuneTargetUp: " + Dashboard.TuneSubsystem.subsystemName);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VisionTest && robot.vision != null)
                {
                    if (pressed)
                    {
                        if (robot.vision.backCamColorBlobVision != null)
                        {
                            // Set display to next intermediate Mat in the pipeline.
                            if (robot.vision.isColorBlobVisionEnabled(Vision.ColorBlobType.Any))
                            {
                                robot.vision.backCamColorBlobVision.getVisionProcessor().getPipeline().setNextVideoOutput();
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
                if (testChoices.test == Test.SubsystemsTest)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            setControlsEnabled(false);
                            tuneDriveMotors(Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorMaxVelocity, 180.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            setControlsEnabled(true);
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        TrcSubsystem.setSubsystemTuneTargetDown(Dashboard.TuneSubsystem.subsystemName);
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> SetTuneTargetDown: " + Dashboard.TuneSubsystem.subsystemName);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
                if (testChoices.test == Test.SubsystemsTest)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            setControlsEnabled(false);
                            tuneDriveMotors(Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorMaxVelocity, 270.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            setControlsEnabled(true);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadRight:
                if (testChoices.test == Test.SubsystemsTest)
                {
                    if (RobotParams.Preferences.tuneDriveBase && robot.robotBase != null)
                    {
                        // We are controlling drive base motors, make sure TeleOp doesn't interfere.
                        if (pressed)
                        {
                            setControlsEnabled(false);
                            tuneDriveMotors(Dashboard.SubsystemDrivebase.driveBaseParams.driveMotorMaxVelocity, 90.0);
                        }
                        else
                        {
                            robot.robotBase.cancel();
                            setControlsEnabled(true);
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.VisionTest && robot.vision != null &&
                         robot.vision.backCamColorBlobVision != null)
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
                if (testChoices.test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            TrcSubsystem.updateSubsystemParamsToDashboard(Dashboard.TuneSubsystem.subsystemName);
                            robot.globalTracer.traceInfo(
                                moduleName,
                                ">>>>> Update Dashboard with subsystem tune params.");
                        }
                        else
                        {
                            TrcSubsystem.updateSubsystemParamsFromDashboard(Dashboard.TuneSubsystem.subsystemName);
                            robot.globalTracer.traceInfo(
                                moduleName,
                                ">>>>> Update subsystem tune params from Dashboard and Start subsystem tuning.");
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TuneDriveBasePid)
                {
                    if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
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
                                        testChoices.driveXTarget*12.0,
                                        testChoices.driveYTarget*12.0,
                                        testChoices.turnTarget));
                            }
                            // In FTC, the PID controllers used by PurePursuit are accessible in Dashboard.
                            // So there is no need to update PurePursuit with new PID coefficients from Dashboard.
                            TrcPose2D drivePoint = tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint;
                            robot.robotBase.purePursuitDrive.start(
                                false,
                                Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveVelocity,
                                Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveAcceleration,
                                Dashboard.SubsystemDrivebase.driveBaseParams.profiledMaxDriveDeceleration,
                                null, drivePoint);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Pid Drive to ", drivePoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        else
                        {
                            robot.robotBase.purePursuitDrive.cancel();
                        }
                        passToTeleOp = false;
                    }
                }
                else if (testChoices.test == Test.VisionTest)
                {
                    if (robot.vision != null)
                    {
                        if (pressed)
                        {
                            fpsMeterEnabled = !fpsMeterEnabled;
                            robot.vision.setFpsMeterEnabled(fpsMeterEnabled);
                            robot.globalTracer.traceInfo(moduleName, "fpsMeterEnabled = %s", fpsMeterEnabled);
                        }
                        passToTeleOp = false;
                    }
                }
                break;

            default:
                break;
        }
        //
        // If the button event was not processed by this method, pass it back to TeleOp.
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
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Operator: " + button + "=" + (pressed? "pressed": "released"));
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
            default:
                break;
        }
        //
        // If the button event was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorButtonEvent(button, pressed);
        }
    }   //operatorButtonEvent

}   //class FtcTest
