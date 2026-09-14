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
import teamcode.subsystems.CrServoArm;
import teamcode.subsystems.DiffyServoWrist;
import teamcode.subsystems.DriveBase;
import teamcode.subsystems.Elevator;
import teamcode.subsystems.Intake;
import teamcode.subsystems.Latch;
import teamcode.subsystems.MotorArm;
import teamcode.subsystems.ServoClaw;
import teamcode.subsystems.ServoExtender;
import teamcode.subsystems.ServoWrist;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.TelescopeArm;
import teamcode.subsystems.Turret;
import teamcode.vision.Vision;
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
    protected static final boolean traceButtonEvents = true;

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    protected RumbleIndicator driverRumble;
    protected RumbleIndicator operatorRumble;
    private double drivePowerScale;
    private double turnPowerScale;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    protected boolean controlsEnabled = false;

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
            TrcDbgTrace.openTraceLog(RobotParams.Robot.logFolderPath, filePrefix);
        }
        // Create and initialize Gamepads.
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setLeftStickInverted(false, true);
        driverGamepad.setRightStickInverted(false, true);

        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);

        if (RobotParams.Preferences.useRumble)
        {
            driverRumble = new RumbleIndicator("DriverRumble", driverGamepad);
            operatorRumble = new RumbleIndicator("OperatorRumble", operatorGamepad);
        }

        drivePowerScale = Dashboard.SubsystemDrivebase.driveNormalScale;
        turnPowerScale = Dashboard.SubsystemDrivebase.turnNormalScale;
        if (robot.robotDriveBase != null)
        {
            robot.robotDriveBase.setDriveOrientation(Dashboard.SubsystemDrivebase.driveOrientation);
        }
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
            if (robot.vision.frontCamAprilTagVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling WebCam AprilTagVision.");
                robot.vision.setWebcamAprilTagVisionEnabled(true);
            }
            else if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling Limelight AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.AprilTag, true);
            }
        }
        //
        // Enabling gamepads.
        //
        setControlsEnabled(true);
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
        // Disabling gamepads.
        //
        setControlsEnabled(false);
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
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

        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase subsystem.
                //
                if (robot.robotBase != null)
                {
                    double[] inputs = driverGamepad.getDriveInputs(
                        Dashboard.SubsystemDrivebase.driveMode, true, drivePowerScale, turnPowerScale);

                    robot.robotDriveBase.subsystemControl(driverAltFunc, inputs);
                    // Check for EndGame warning.
                    if (elapsedTime > RobotParams.Game.ENDGAME_THRESHOLD)
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
                    // Note that this sample code assumes only one subsystem is enabled at a time for demo purpose.
                    // Therefore, the same control may be assigned to multiple subsystems.
                    if (robot.motorArmSubsystem != null)
                    {
                        robot.motorArmSubsystem.subsystemControl(driverAltFunc, driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.crServoArmSubsystem != null)
                    {
                        robot.crServoArmSubsystem.subsystemControl(driverAltFunc, driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.telescopeArm != null)
                    {
                        robot.telescopeArm.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickY(true), driverGamepad.getRightStickY(true));
                    }
                    else if (robot.elevatorSubsystem != null)
                    {
                        robot.elevatorSubsystem.subsystemControl(driverAltFunc, driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.turretSubsystem != null)
                    {
                        robot.turretSubsystem.subsystemControl(driverAltFunc, driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.shooterSubsystem != null)
                    {
                        robot.shooterSubsystem.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickY(true), driverGamepad.getRightStickX(true));
                    }
                    else if (robot.diffyWrist != null)
                    {
                        robot.diffyWrist.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickX(true), driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.servoWristSubsystem != null)
                    {
                        robot.servoWristSubsystem.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickX(true), driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.servoExtenderSubsystem != null)
                    {
                        robot.servoExtenderSubsystem.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickX(true), driverGamepad.getLeftStickY(true));
                    }
                    else if (robot.latchSubsystem != null)
                    {
                        robot.latchSubsystem.subsystemControl(
                            driverAltFunc, driverGamepad.getLeftStickX(true), driverGamepad.getLeftStickY(true));
                    }
                }
            }
        }
    }   //periodic

    /**
     * This method enables/disables gamepad controls.
     *
     * @param enabled specifies true to enable gamepad controls, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;
        if (enabled)
        {
            driverGamepad.setButtonEventHandler(this::driverButtonEvent);
            operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        }
        else
        {
            driverGamepad.setButtonEventHandler(null);
            operatorGamepad.setButtonEventHandler(null);
        }
    }   //setControlsEnabled

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
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Driver: " + button + "=" + (pressed? "pressed": "released"));
        switch (button)
        {
            case A:
                if (robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        if (robot.autoShootTask != null)
                        {
                            robot.shooterSubsystem.subsystemAction(Shooter.Action.ToggleAutoShoot, !driverAltFunc);
                        }
                        else
                        {
                            robot.shooterSubsystem.subsystemAction(Shooter.Action.ToggleManualShoot, null);
                        }
                    }
                }
                else if (robot.intakeSubsystem != null)
                {
                    if (pressed)
                    {
                        if (robot.autoPickupTask != null)
                        {
                            robot.intakeSubsystem.subsystemAction(Intake.Action.ToggleAutoPickup, !driverAltFunc);
                        }
                        else
                        {
                            if (driverAltFunc)
                            {
                                robot.intakeSubsystem.subsystemAction(Intake.Action.ToggleManualIntake, null);
                            }
                            else
                            {
                                robot.intakeSubsystem.subsystemAction(Intake.Action.ToggleSensorIntake, null);
                            }
                        }
                    }
                }
                else if (robot.servoExtenderSubsystem != null)
                {
                    robot.servoExtenderSubsystem.subsystemAction(ServoExtender.Action.TogglePos, null);
                }
                else if (robot.servoClawSubsystem != null)
                {
                    robot.servoClawSubsystem.subsystemAction(ServoClaw.Action.TogglePos, null);
                }
                else if (robot.latchSubsystem != null)
                {
                    robot.latchSubsystem.subsystemAction(Latch.Action.TogglePos, null);
                }
                break;

            case B:
                if (robot.robotDriveBase != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            robot.robotDriveBase.subsystemAction(DriveBase.Action.ToggleGyroAssist, null);
                        }
                        else
                        {
                            robot.robotDriveBase.subsystemAction(DriveBase.Action.ToggleDriveMode, null);
                        }
                    }
                }
                break;

            case X:
            case Y:
                break;

            case LeftBumper:
                driverAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + driverAltFunc);
                break;

            case RightBumper:
                setDriveSpeedMode(pressed, driverAltFunc);
                break;

            case DpadUp:
                if (robot.motorArmSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.motorArmSubsystem.subsystemAction(MotorArm.Action.PresetPosUp, null);
                    }
                }
                else if (robot.crServoArmSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.crServoArmSubsystem.subsystemAction(CrServoArm.Action.PresetPosUp, null);
                    }
                }
                else if (robot.telescopeArm != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            robot.telescopeArm.subsystemAction(
                                TelescopeArm.Action.PresetPosUp, TelescopeArm.ElbowParams.MOTOR_NAME);
                        }
                        else
                        {
                            robot.telescopeArm.subsystemAction(
                                TelescopeArm.Action.PresetPosUp, TelescopeArm.TelescopeParams.MOTOR_NAME);
                        }
                    }
                }
                else if (robot.elevatorSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.elevatorSubsystem.subsystemAction(Elevator.Action.PresetPosUp, null);
                    }
                }
                else if (robot.turretSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.turretSubsystem.subsystemAction(Turret.Action.PresetPosUp, null);
                    }
                }
                else if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.subsystemAction(DiffyServoWrist.Action.TiltPresetPosUp, null);
                    }
                }
                else if (robot.servoWristSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.servoWristSubsystem.subsystemAction(ServoWrist.Action.PresetPosUp, null);
                    }
                }
                else if (robot.servoExtenderSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.servoExtenderSubsystem.subsystemAction(ServoExtender.Action.PresetPosUp, null);
                    }
                }
                else if (robot.latchSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.latchSubsystem.subsystemAction(Latch.Action.PresetPosUp, null);
                    }
                }
                else if (robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.subsystemAction(Shooter.Action.IncShooterVelocity, null);
                    }
                }
                break;

            case DpadDown:
                if (robot.motorArmSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.motorArmSubsystem.subsystemAction(MotorArm.Action.PresetPosDown, null);
                    }
                }
                else if (robot.crServoArmSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.crServoArmSubsystem.subsystemAction(CrServoArm.Action.PresetPosDown, null);
                    }
                }
                else if (robot.telescopeArm != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            robot.telescopeArm.subsystemAction(
                                TelescopeArm.Action.PresetPosDown, TelescopeArm.ElbowParams.MOTOR_NAME);
                        }
                        else
                        {
                            robot.telescopeArm.subsystemAction(
                                TelescopeArm.Action.PresetPosDown, TelescopeArm.TelescopeParams.MOTOR_NAME);
                        }
                    }
                }
                else if (robot.elevatorSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.elevatorSubsystem.subsystemAction(Elevator.Action.PresetPosDown, null);
                    }
                }
                else if (robot.turretSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.turretSubsystem.subsystemAction(Turret.Action.PresetPosDown, null);
                    }
                }
                else if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.subsystemAction(DiffyServoWrist.Action.TiltPresetPosDown, null);
                    }
                }
                else if (robot.servoWristSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.servoWristSubsystem.subsystemAction(ServoWrist.Action.PresetPosDown, null);
                    }
                }
                else if (robot.servoExtenderSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.servoExtenderSubsystem.subsystemAction(ServoExtender.Action.PresetPosDown, null);
                    }
                }
                else if (robot.latchSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.latchSubsystem.subsystemAction(Latch.Action.PresetPosDown, null);
                    }
                }
                else if (robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.subsystemAction(Shooter.Action.DecShooterVelocity, null);
                    }
                }
                break;

            case DpadLeft:
                if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.subsystemAction(DiffyServoWrist.Action.RotatePresetPosDown, null);
                    }
                }
                else if (robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.subsystemAction(Shooter.Action.DecShooterVelIncrement, null);
                    }
                }
                break;

            case DpadRight:
                if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.subsystemAction(DiffyServoWrist.Action.RotatePresetPosUp, null);
                    }
                }
                else if (robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.subsystemAction(Shooter.Action.IncShooterVelIncrement, null);
                    }
                }
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
                if (pressed)
                {
                    robot.setRelocalizationMode(
                        driverAltFunc? Robot.RelocalizationMode.Continuous: Robot.RelocalizationMode.OneShot);
                }
                break;

            default:
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
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + operatorAltFunc);
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
                if (pressed)
                {
                    if (operatorAltFunc)
                    {
                        Dashboard.DashboardParams.alliance =
                            Dashboard.DashboardParams.alliance == FtcAuto.Alliance.Blue?
                                FtcAuto.Alliance.Red: FtcAuto.Alliance.Blue;
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Toggle alliance: alliance=" + Dashboard.DashboardParams.alliance);
                    }
                    else
                    {
                        robot.cancelAll();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All");
                    }
                }
                break;

            default:
                break;
        }
    }   //operatorButtonEvent

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
                drivePowerScale = Dashboard.SubsystemDrivebase.driveSlowScale;
                turnPowerScale = Dashboard.SubsystemDrivebase.turnSlowScale;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
            }
            else
            {
                drivePowerScale = Dashboard.SubsystemDrivebase.driveNormalScale;
                turnPowerScale = Dashboard.SubsystemDrivebase.turnNormalScale;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
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
     * This method is called to zero calibrate all subsystems.
     */
    private void zeroCalibrate()
    {
        // Cancel all operations and zero calibrate all subsystems (arm, elevator and turret).
        robot.cancelAll();
        robot.zeroCalibrate(null, null);
        robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrating.");
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
