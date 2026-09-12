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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

import ftclib.driverio.FtcChoiceMenu;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.driverio.FtcMenu;
import ftclib.driverio.FtcValueMenu;
import ftclib.robotcore.FtcOpMode;
import teamcode.autocommands.CmdAutoStartPosLeft;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the Autonomous Mode program.
 */
@Autonomous(name="FtcAutonomous", group="Ftc####", preselectTeleOp="FtcTeleOp")
public class FtcAuto extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();
    //
    // Auto choices enums.
    //
    public enum Alliance
    {
        Red,
        Blue
    }   //enum Alliance

    public enum AutoStartPos
    {
        Left,
        Right
    }   //enum AutoStartPos

    public enum AutoStrategy
    {
        StartPosLeftAuto,
        PurePursuitDrive,
        PidDrive,
        TimedDrive,
        DoNothing
    }   //enum AutoStrategy

    public enum PurePursuitDrivePath
    {
        Path1,
        Path2
    }   //enum PurePursuitDrivePath

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        // Standard choice menus.
        private final FtcChoiceMenu<Alliance> allianceMenu;
        private final FtcChoiceMenu<AutoStartPos> startPosMenu;
        private final FtcValueMenu startDelayMenu;
        private final FtcChoiceMenu<AutoStrategy> strategyMenu;
        private final FtcChoiceMenu<PurePursuitDrivePath> ppDrivePathMenu;
        private final FtcValueMenu xTargetMenu;
        private final FtcValueMenu yTargetMenu;
        private final FtcValueMenu turnTargetMenu;
        private final FtcValueMenu drivePowerMenu;
        private final FtcValueMenu turnPowerMenu;
        private final FtcValueMenu timedDrivePowerMenu;
        private final FtcValueMenu timedDriveTimeMenu;
        // Game specific choice menus.
        private final FtcChoiceMenu<Boolean> useVisionMenu;
        private final FtcChoiceMenu<Boolean> scorePreloadMenu;
        // Standard auto choices.
        public Alliance alliance = null;
        public AutoStartPos startPos = null;
        public AutoStrategy strategy = null;
        public double startDelay = 0.0;
        public double xDriveDistance = 0.0;
        public double yDriveDistance = 0.0;
        public double turnAngle = 0.0;
        public double drivePower = 0.0;
        public double turnPower = 0.0;
        public double timedDrivePower = 0.0;
        public double timedDriveTime = 0.0;
        public String purePursuitPathFile = null;
        // Game specific choices.
        public boolean useVision = false;
        public boolean scorePreload = false;

        public AutoChoices()
        {
            //
            // Construct menus.
            //
            // Standard choice menus.
            allianceMenu = new FtcChoiceMenu<>("Alliance:", null);
            startPosMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu);
            startDelayMenu = new FtcValueMenu("Start delay:", startPosMenu, 0.0, 30.0, 1.0, 0.0, " %.0f sec");
            strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", startDelayMenu);
            ppDrivePathMenu = new FtcChoiceMenu<>("PurePursuit Drive Path:", strategyMenu);
            xTargetMenu = new FtcValueMenu("xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
            yTargetMenu = new FtcValueMenu("yTarget:", xTargetMenu, -12.0, 12.0, 0.5, 4.0, " %.1f ft");
            turnTargetMenu = new FtcValueMenu("turnTarget:", yTargetMenu, -180.0, 180.0, 5.0, 90.0, " %.0f deg");
            drivePowerMenu = new FtcValueMenu("Drive power:", turnTargetMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            turnPowerMenu = new FtcValueMenu("Turn power:", drivePowerMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            timedDrivePowerMenu = new FtcValueMenu("Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
            timedDriveTimeMenu = new FtcValueMenu("Drive time:", timedDrivePowerMenu, 0.0, 30.0, 1.0, 5.0, " %.0f sec");
            // Game specific choice menus.
            useVisionMenu = new FtcChoiceMenu<>("Use Vision:", strategyMenu);
            scorePreloadMenu = new FtcChoiceMenu<>("Score Preloads:", useVisionMenu);
            //
            // Populate choice menus.
            //
            allianceMenu.addChoice("Red", Alliance.Red, true, startPosMenu);
            allianceMenu.addChoice("Blue", Alliance.Blue, false, startPosMenu);

            startPosMenu.addChoice("Start Position Left", AutoStartPos.Left, true, strategyMenu);
            startPosMenu.addChoice("Start Position Right", AutoStartPos.Right, false, strategyMenu);

            strategyMenu.addChoice("Left Auto", AutoStrategy.StartPosLeftAuto, false, useVisionMenu);
            strategyMenu.addChoice("PurePursuit Drive", AutoStrategy.PurePursuitDrive, false, xTargetMenu);
            strategyMenu.addChoice("PID Drive", AutoStrategy.PidDrive, false, xTargetMenu);
            strategyMenu.addChoice("Timed Drive", AutoStrategy.TimedDrive, false, timedDrivePowerMenu);
            strategyMenu.addChoice("Do nothing", AutoStrategy.DoNothing, true);

            ppDrivePathMenu.addChoice("Path 1", PurePursuitDrivePath.Path1, true);
            ppDrivePathMenu.addChoice("Path 2", PurePursuitDrivePath.Path2, false);

            // Game specific choices.
            useVisionMenu.addChoice("true", Boolean.TRUE, true, scorePreloadMenu);
            useVisionMenu.addChoice("false", Boolean.FALSE, false, scorePreloadMenu);

            scorePreloadMenu.addChoice("true", Boolean.TRUE, true);
            scorePreloadMenu.addChoice("false", Boolean.FALSE, false);
            //
            // Link Value Menus to their children.
            //
            startDelayMenu.setChildMenu(strategyMenu);
            xTargetMenu.setChildMenu(yTargetMenu);
            yTargetMenu.setChildMenu(turnTargetMenu);
            turnTargetMenu.setChildMenu(drivePowerMenu);
            drivePowerMenu.setChildMenu(turnPowerMenu);
            timedDrivePowerMenu.setChildMenu(timedDriveTimeMenu);
        }   //AutoChoices

        /**
         * This method displays the Auto Choice menus for selection and stores the choices.
         */
        private void fetchChoices(FtcDashboard dashboard)
        {
            //
            // Traverse menus.
            //
            FtcMenu.walkMenuTree(allianceMenu);
            //
            // Fetch choices.
            //
            alliance = allianceMenu.getCurrentChoiceObject();
            startPos = startPosMenu.getCurrentChoiceObject();
            startDelay = startDelayMenu.getCurrentValue();
            strategy = strategyMenu.getCurrentChoiceObject();
            xDriveDistance = xTargetMenu.getCurrentValue();
            yDriveDistance = yTargetMenu.getCurrentValue();
            turnAngle = turnTargetMenu.getCurrentValue();
            drivePower = drivePowerMenu.getCurrentValue();
            turnPower = turnPowerMenu.getCurrentValue();
            timedDrivePower = timedDrivePowerMenu.getCurrentValue();
            timedDriveTime = timedDriveTimeMenu.getCurrentValue();
            switch (ppDrivePathMenu.getCurrentChoiceObject())
            {
                case Path1:
                    purePursuitPathFile = RobotParams.Robot.purePursuitPathFile1;
                    break;

                case Path2:
                    purePursuitPathFile = RobotParams.Robot.purePursuitPathFile2;
                    break;

                default:
                    purePursuitPathFile = null;
                    break;
            }
            // Game specific choices.
            useVision = useVisionMenu.getCurrentChoiceObject();
            scorePreload = scorePreloadMenu.getCurrentChoiceObject();

            // Update Dashboard with AutoChoice alliance.
            Dashboard.DashboardParams.alliance = autoChoices.alliance;
            //
            // Show choices.
            //
            if (dashboard != null)
            {
                dashboard.displayPrintf(1, "Auto Choices: %s", autoChoices);
            }
        }   //fetchChoices

        @NonNull
        @Override
        public String toString()
        {
            return "alliance=\"" + alliance + "\" " +
                   "startPos=\"" + startPos + "\" " +
                   "strategy=\"" + strategy + "\" " +
                   "startDelay=" + startDelay + " sec " +
                   "xDistance=" + xDriveDistance + " ft " +
                   "yDistance=" + yDriveDistance + " ft " +
                   "turnDegrees=" + turnAngle + " deg " +
                   "drivePower=" + drivePower + "\" " +
                   "turnPower=" + turnPower + "\" " +
                   "timedDrivePower=" + timedDrivePower + "\" " +
                   "timedDriveTime=" + timedDriveTime + " sec " +
                   "ppPathFile=\"" + purePursuitPathFile + "\" " +
                   // Game specific choices.
                   "useVision=" + useVision + " " +
                   "scorePreload=" + scorePreload;
        }   //toString
    }   //class AutoChoices

    //
    // Global objects.
    //
    public static final AutoChoices autoChoices = new AutoChoices();
    private Robot robot;
    private TrcRobot.RobotCommand autoCommand;

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
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            Robot.matchInfo = FtcMatchInfo.getMatchInfo();
            String filePrefix = String.format(
                Locale.US, "%s%02d_Auto", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber);
            TrcDbgTrace.openTraceLog(RobotParams.Robot.logFolderPath, filePrefix);
        }
        autoChoices.fetchChoices(robot.dashboard);
        //
        // Create autonomous command according to chosen strategy.
        //
        switch (autoChoices.strategy)
        {
            case StartPosLeftAuto:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdAutoStartPosLeft(robot, autoChoices);
                }
                break;

            case PurePursuitDrive:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    autoCommand = new CmdPurePursuitDrive(
                        robot.robotBase.driveBase, robot.robotInfo.baseParams.xDrivePidCoeffs,
                        robot.robotInfo.baseParams.yDrivePidCoeffs, robot.robotInfo.baseParams.turnPidCoeffs,
                        robot.robotInfo.baseParams.velPidCoeffs);
                }
                break;

            case PidDrive:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    autoCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                }
                break;

            case TimedDrive:
                if (robot.robotBase != null)
                {
                    // TimedDrive only goes in the Y direction. Set up the robot to aim where you want to go.
                    autoCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, autoChoices.startDelay, autoChoices.timedDriveTime, 0.0,
                        autoChoices.timedDrivePower, 0.0);
                }
                break;

            case DoNothing:
            default:
                autoCommand = null;
                break;
        }

//        if (robot.vision != null)
//        {
//            // If necessary, enable vision early so we can detect target before match starts.
//            // Only enable the necessary vision for that purpose.
//            if (robot.vision.limelightVision != null)
//            {
//                robot.globalTracer.traceInfo(moduleName, "Enabling LimelightAprilTagVision.");
//                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.AprilTag, true);
//            }
//
//            if (robot.vision.frontCamAprilTagVision != null)
//            {
//                robot.globalTracer.traceInfo(moduleName, "Enabling frontCamAprilTagVision.");
//                robot.vision.setWebcamAprilTagVisionEnabled(true);
//            }
//
//            if (robot.vision.backCamColorBlobVision != null)
//            {
//                robot.globalTracer.traceInfo(moduleName, "Enabling backCamColorBlobVision.");
//                robot.vision.setColorBlobVisionEnabled(
//                    autoChoices.alliance == Alliance.Red? Vision.ColorBlobType.RedBlob: Vision.ColorBlobType.BlueBlob,
//                    true);
//            }
//        }
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically after robotInit() is called but before competition starts. For example,
     * we can put vision code here to detect target before autonomous starts, or code to move the subsystems to a
     * particular configuration to be within the 18-inch starting restriction.
     */
    @Override
    public void initPeriodic()
    {
    }   //initPeriodic

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
            moduleName, "***** Starting autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        if (Robot.matchInfo != null)
        {
            robot.globalTracer.logInfo(moduleName, "MatchInfo", Robot.matchInfo.toString());
        }
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }

        if (autoCommand != null)
        {
            if (autoChoices.strategy == AutoStrategy.PurePursuitDrive)
            {
                ((CmdPurePursuitDrive) autoCommand).startPath(
                    0.0, false,
                    robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                    robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                    robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                    autoChoices.drivePower, autoChoices.turnPower,
                    autoChoices.purePursuitPathFile, false);
            }
            else if (autoChoices.strategy == AutoStrategy.PidDrive)
            {
                ((CmdPidDrive) autoCommand).startPath(
                    autoChoices.startDelay, autoChoices.drivePower, autoChoices.turnPower, null,
                    new TrcPose2D(
                        autoChoices.xDriveDistance*12.0,
                        autoChoices.yDriveDistance*12.0,
                        autoChoices.turnAngle));
            }

            autoCommand.start();
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
        // OpMode is about to stop, cancel autonomous command in progress if any.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);

        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }

        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping autonomous: " + TrcTimer.getCurrentTimeString() + " *****");
        printPerformanceMetrics();

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog(null);
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
        robot.periodic(elapsedTime, slowPeriodicLoop);
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //periodic

}   //class FtcAuto
