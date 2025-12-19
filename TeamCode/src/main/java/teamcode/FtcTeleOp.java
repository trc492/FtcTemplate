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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import ftclib.drivebase.FtcSwerveBase;
import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import teamcode.indicators.RumbleIndicator;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftc####")
public class FtcTeleOp extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    protected RumbleIndicator driverRumble;
    protected RumbleIndicator operatorRumble;
    private double drivePowerScale;
    private double turnPowerScale;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    protected boolean allowAnalogControl = true;
    private boolean relocalizing = false;
    private int relocalizeCount = 0;
    private Integer savedLimelightPipeline = null;

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
        // Create and initialize robot object.
        robot = new Robot(TrcRobot.getRunMode());
        // Open trace log.
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Standalone_TeleOp";
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        // Create and initialize Gamepads.
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setButtonEventHandler(this::driverButtonEvent);
        driverGamepad.setLeftStickInverted(false, true);
        driverGamepad.setRightStickInverted(false, true);

        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        operatorGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);

        if (RobotParams.Preferences.useRumble)
        {
            driverRumble = new RumbleIndicator("DriverRumble", driverGamepad);
            operatorRumble = new RumbleIndicator("OperatorRumble", operatorGamepad);
        }

        drivePowerScale = Dashboard.Subsystem_Drivebase.driveNormalScale;
        turnPowerScale = Dashboard.Subsystem_Drivebase.turnNormalScale;
        setDriveOrientation(Dashboard.Subsystem_Drivebase.driveOrientation);
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
        if (robot.vision != null)
        {
            if (robot.vision.webcamAprilTagVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling WebCam AprilTagVision.");
                robot.vision.setWebcamAprilTagVisionEnabled(true);
            }
            else if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling Limelight AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
            }
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
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        printPerformanceMetrics();

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
        robot.periodic(elapsedTime, slowPeriodicLoop);
        if (allowAnalogControl)
        {
            if (slowPeriodicLoop)
            {
                //
                // DriveBase subsystem.
                //
                if (robot.robotBase != null)
                {
                    // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                    if (relocalizing)
                    {
                        // Relocalize twice, using MT1 for the first time and MT2 for the second time.
                        if (relocalizeCount < 2)
                        {
                            TrcPose2D robotFieldPose = robot.vision.getRobotFieldPose();
                            if (robotFieldPose != null)
                            {
                                robot.relocalizedRobotPose = robotFieldPose;
                                // Vision found an AprilTag, set the new robot field location.
                                robot.globalTracer.traceInfo(
                                    moduleName,
                                    ">>>>> Relocalizing: pose=" + robot.relocalizedRobotPose);
                                robot.robotBase.driveBase.setFieldPosition(robot.relocalizedRobotPose, false);
                                relocalizeCount++;
                            }
                        }
                    }
                    else
                    {
                        double[] inputs = driverGamepad.getDriveInputs(
                            Dashboard.Subsystem_Drivebase.driveMode, true, drivePowerScale, turnPowerScale);

                        if (robot.robotBase.driveBase.supportsHolonomicDrive())
                        {
                            robot.robotBase.driveBase.holonomicDrive(
                                null, inputs[0], inputs[1], inputs[2], robot.robotBase.driveBase.getDriveGyroAngle());
                        }
                        else
                        {
                            robot.robotBase.driveBase.arcadeDrive(inputs[1], inputs[2]);
                        }

                        if (robot.dashboard.isDashboardUpdateEnabled() && RobotParams.Preferences.showDriveBaseStatus)
                        {
                            robot.dashboard.displayPrintf(
                                14, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                                inputs[0], inputs[1], inputs[2], robot.robotBase.driveBase.getDriveOrientation());
                        }
                    }
                    // Check for EndGame warning.
                    if (elapsedTime > RobotParams.Game.ENDGAME_DEADLINE)
                    {
                        if (driverRumble != null)
                        {
                            driverRumble.setRumblePattern(RumbleIndicator.ENDGAME_DEADLINE);
                        }

                        if (operatorRumble != null)
                        {
                            operatorRumble.setRumblePattern(RumbleIndicator.ENDGAME_DEADLINE);
                        }
                    }
                }
                //
                // Other subsystems.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    // Analog control of subsystems.
                }
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    private void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotBase != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=" + orientation);
            robot.robotBase.driveBase.setDriveOrientation(orientation, false);
            if (orientation == TrcDriveBase.DriveOrientation.FIELD)
            {
                robot.robotBase.driveBase.setFieldForwardHeading(
                    Dashboard.DashboardParams.alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0);
            }
            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setDriveOrientation(orientation);
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
        robot.dashboard.displayPrintf(15, "Driver: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
            case B:
                break;

            case X:
                if (pressed)
                {
                    setDriveMode(driverAltFunc);
                }
                break;

            case Y:
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + pressed);
                driverAltFunc = pressed;
                break;

            case RightBumper:
                setDriveSpeedMode(pressed, driverAltFunc);
                break;

            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    if (!driverAltFunc)
                    {
                        zeroCalibrate();
                    }
                    else
                    {
                        resetSwerveSteering();
                    }
                }
                break;

            case Start:
                relocalize(pressed);
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
        robot.dashboard.displayPrintf(15, "Operator: %s=%s", button, pressed? "Pressed": "Released");

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
                if (pressed && operatorAltFunc)
                {
                    toggleDashboardUpdateMode();
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
                    zeroCalibrate();
                }
                break;

            case Start:
                if (operatorAltFunc && pressed)
                {
                    Dashboard.DashboardParams.alliance =
                        Dashboard.DashboardParams.alliance == FtcAuto.Alliance.BLUE_ALLIANCE?
                            FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
                }
                break;
        }
    }   //operatorButtonEvent

    /**
     * This method is called to change the drive mode between ROBOT mode and FIELD mode or turning ON/OFF GyroAssist.
     *
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setDriveMode(boolean altFunc)
    {
        if (robot.robotBase != null)
        {
            if (altFunc)
            {
                if (robot.robotBase.driveBase.isGyroAssistEnabled())
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
                    robot.robotBase.driveBase.setGyroAssistEnabled(null);
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
                    robot.robotBase.driveBase.setGyroAssistEnabled(
                        robot.robotBase.purePursuitDrive.getTurnPidCtrl());
                }
            }
            else if (robot.robotBase.driveBase.supportsHolonomicDrive())
            {
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (robot.robotBase.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD)
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
    }   //setDriveMode

    /**
     * This method is called to set drive speed modes.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setDriveSpeedMode(boolean pressed, boolean altFunc)
    {
        if (!altFunc)
        {
            // Press and hold for slow drive.
            if (pressed)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
                drivePowerScale = Dashboard.Subsystem_Drivebase.driveSlowScale;
                turnPowerScale = Dashboard.Subsystem_Drivebase.turnSlowScale;
            }
            else
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
                drivePowerScale = Dashboard.Subsystem_Drivebase.driveNormalScale;
                turnPowerScale = Dashboard.Subsystem_Drivebase.turnNormalScale;
            }
        }
        else
        {
            if (pressed)
            {
                toggleDashboardUpdateMode();
            }
        }
    }   //setDriveSpeedMode

    /**
     * This method is called to relocalize the robot.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     */
    private void relocalize(boolean pressed)
    {
        // Do AprilTag Vision re-localization.
        if (robot.vision != null && robot.robotBase != null)
        {
            boolean hasAprilTagVision = robot.vision.isWebcamAprilTagVisionEnabled();
            // If Webcam AprilTag vision is not enabled, check if we have Limelight since Limelight has
            // AprilTag pipeline as well.
            if (!hasAprilTagVision && robot.vision.limelightVision != null)
            {
                hasAprilTagVision = true;
                if (pressed)
                {
                    // Webcam AprilTag vision is not enable, enable Limelight AprilTag pipeline instead.
                    // Note: we assume pipeline 0 is the AprilTag pipeline.
                    savedLimelightPipeline = robot.vision.limelightVision.getPipeline();
                    robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
                }
            }

            if (hasAprilTagVision)
            {
                relocalizing = pressed;
                // On press of the button, we will start looking for AprilTag for re-localization.
                // On release of the button, we will set the robot's field location if we found the AprilTag.
                if (pressed)
                {
                    relocalizeCount = 0;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                }
                else if (savedLimelightPipeline != null)
                {
                    // Done with AprilTag re-localization, restore previous Limelight pipeline.
                    robot.vision.limelightVision.setPipeline(savedLimelightPipeline);
                    savedLimelightPipeline = null;
                }
            }
        }
    }   //relocalize

    /**
     * This method is called to zero calibrate all subsystems.
     */
    private void zeroCalibrate()
    {
        // Cancel all operations and zero calibrate all subsystems (arm, elevator and turret).
        robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrating.");
        robot.cancelAll();
        robot.zeroCalibrate(null, null);
    }   //zeroCalibrate

    /**
     * This method is called to set all swerve steering to zero angle.
     */
    private void resetSwerveSteering()
    {
        // If drive base is SwerveDrive, set all wheels pointing forward.
        if (robot.robotBase != null && robot.robotBase instanceof FtcSwerveBase)
        {
            // Drive base is a Swerve Drive, align all steering wheels forward.
            robot.globalTracer.traceInfo(moduleName, ">>>>> Set SteerAngle to zero.");
            ((FtcSwerveBase) robot.robotBase).setSteerAngle(0.0, false, false);
        }
    }   //resetSwerveSteering

    /**
     * This method is called to toggle DashboardUpdate mode.
     */
    private void toggleDashboardUpdateMode()
    {
        boolean enabled = !robot.dashboard.isDashboardUpdateEnabled();
        robot.globalTracer.traceInfo(moduleName, ">>>>> setUpdateDashboardEnable=" + enabled);
        if (enabled)
        {
            robot.dashboard.enableDashboardUpdate(1, true);
        }
        else
        {
            robot.dashboard.disableDashboardUpdate();
        }
    }   //toggleDashboardUpdateMode

}   //class FtcTeleOp
