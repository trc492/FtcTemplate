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

package teamcode.subsystems;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.motor.FtcServoActuator;
import ftclib.subsystem.FtcShooter;
import teamcode.Dashboard;
import teamcode.Robot;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcDiscreteValue;
import trclib.dataprocessor.TrcLookupTable;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Shooter Subsystem. This implementation consists of one or two shooter motors. For
 * two-motor shooter, the two motors can be arranged to spin in the same direction (2-stage shooting) or in opposite
 * directions. For opposite spinning motor arrangement, one can spin the motors at different speed to create back spin
 * when shooting the object. In the two-motor configuration, because the two motors may not be identical (even if they
 * are the same model), the subsystem allows you to tune different PID coefficients for each motor. The shooter
 * subsystem also supports optionally mounting on a pan and tilt platform. This allows for aiming the shooter at
 * the shooting target.
 */
public class Shooter extends TrcSubsystem<Shooter.Action>
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = false;
    private static final double GOBILDA6000_CPR = 28.0;

    private static final boolean HAS_PAN_MOTOR = false;
    private static final boolean HAS_TILT_MOTOR = false;
    private static final boolean HAS_LAUNCHER = false;

    public static class ShooterMotorParams
    {
        private static final boolean HAS_TWO_SHOOTER_MOTORS     = false;
        // Shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;

        public static final String MOTOR1_NAME                  = SUBSYSTEM_NAME + ".shooterMotor1";
        public static final boolean MOTOR1_INVERTED             = false;
        public static final boolean MOTOR1_VOLTCOMP_ENABLED     = true;
        public static final boolean MOTOR1_BRAKE_ENABLED        = false;

        public static final TrcPidController.PidCoefficients motor1PidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0085, 0.0);

        public static final String MOTOR2_NAME                  = SUBSYSTEM_NAME + ".shooterMotor2";
        public static final boolean MOTOR2_INVERTED             = true;
        public static final boolean MOTOR2_VOLTCOMP_ENABLED     = true;
        public static final boolean MOTOR2_BRAKE_ENABLED        = false;

        public static final TrcPidController.PidCoefficients motor2PidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0085, 0.0);

        public static final double PID_TOLERANCE                = 1.0;      // in RPS (60 RPM)
        public static final boolean USE_SOFTWARE_PID            = true;

        public static final double GEAR_RATIO                   = 24.0/36.0;
        public static final double REV_PER_COUNT                = 1.0/(GOBILDA6000_CPR * GEAR_RATIO);

        public static final double OFF_DELAY                    = 0.5;      // in sec

        // These are for tuning shooter motor with gamepad.
        public static final double MIN_VEL                      = 10.0;     // in RPM
        public static final double MAX_VEL                      = 7360.0;   // in RPM
        public static final double MIN_VEL_INC                  = 1.0;      // in RPM
        public static final double MAX_VEL_INC                  = 1000.0;   // in RPM
        public static final double DEF_VEL                      = 1000.0;   // in RPM
        public static final double DEF_VEL_INC                  = 100.0;    // in RPM
    }   //class ShooterMotorParams

    public static class PanMotorParams
    {
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".panMotor";
        public static final boolean MOTOR_INVERTED              = false;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final double PID_TOLERANCE                = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients pidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);

        public static final double DEG_PER_COUNT                = 1.0;
        public static final double POS_OFFSET                   = -90.0;
        public static final double MIN_POS                      = -90.0;
        public static final double FRONT_POS                    = 0.0;
        public static final double MAX_POS                      = 90.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, -60.0, -30.0, 0.0, 30.0, 60.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.2;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class PanMotorParams

    public static class TiltMotorParams
    {
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".tiltMotor";
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = false;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final double PID_TOLERANCE                = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients pidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);

        public static final double DEG_PER_COUNT                = 1.0;
        public static final double POS_OFFSET                   = 0.0;
        public static final double MIN_POS                      = 0.0;
        public static final double MAX_POS                      = 90.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 30.0, 45.0, 60.0, 75.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
    }   //class TiltMotorParams

    public static class LauncherParams
    {
        public static final String SERVO_NAME                   = SUBSYSTEM_NAME + ".launcher";
        public static final boolean SERVO_INVERTED              = true;
        public static double REST_POS                           = 0.0;
        public static double LAUNCH_POS                         = 0.5;
        public static double LAUNCH_DURATION                    = 0.5;  // in seconds
    }   //class LauncherParams

    public enum Action
    {
        ToggleAutoShoot,
        ToggleManualShoot,
        IncShooterVelocity,
        DecShooterVelocity,
        IncShooterVelIncrement,
        DecShooterVelIncrement
    }   //enum Action

    public static final TrcPose2D robotToShooterPose = new TrcPose2D(0.0, 0.0, 0.0);

    private static final TrcLookupTable.Region[] regions =
    {
        new TrcLookupTable.Region(60.0, new double[][] {null})
    };

    public static final TrcLookupTable shootParamTable = new TrcLookupTable()
        //        name,                 distance,   region,             ShooterVel(rps)
        .addEntry(null,                 36.0,       regions[0],         60.0)
        .addEntry(null,                 48.0,       regions[0],         70.0)
        .addEntry(null,                 60.0,       regions[0],         80.0)
        .addEntry(null,                 72.0,       regions[0],         90.0);

    private final Robot robot;
    private final FtcDashboard dashboard;
    private final TrcShooter shooter;
    public final TrcDiscreteValue shooter1Velocity;
    public final TrcDiscreteValue shooter2Velocity;
    public final TrcServo launcher;

    private String launchOwner;
    private TrcEvent launchCompletionEvent;
    private TrcEvent launchCallbackEvent = null;

    private String tuneSubsystemName = null;
    private double prevTiltPower = 0.0;
    private double prevPanPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access other subsystems if necessary.
     */
    public Shooter(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.robot = robot;
        dashboard = FtcDashboard.getInstance();
        FtcShooter.Params shooterParams = new FtcShooter.Params()
            .setShooterMotor1(
                ShooterMotorParams.MOTOR1_NAME, ShooterMotorParams.MOTOR_TYPE, ShooterMotorParams.MOTOR1_INVERTED,
                ShooterMotorParams.MOTOR1_VOLTCOMP_ENABLED, ShooterMotorParams.MOTOR1_BRAKE_ENABLED,
                false);

        if (ShooterMotorParams.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                ShooterMotorParams.MOTOR2_NAME, ShooterMotorParams.MOTOR_TYPE, ShooterMotorParams.MOTOR2_INVERTED,
                ShooterMotorParams.MOTOR2_VOLTCOMP_ENABLED, ShooterMotorParams.MOTOR2_BRAKE_ENABLED,
                false, true);
        }

        if (HAS_PAN_MOTOR)
        {
            shooterParams.setPanMotor(
                PanMotorParams.MOTOR_NAME, PanMotorParams.MOTOR_TYPE, PanMotorParams.MOTOR_INVERTED,
                PanMotorParams.MOTOR_VOLTCOMP_ENABLED, PanMotorParams.MOTOR_BRAKE_ENABLED, null, false, false,
                new TrcShooter.PanTiltParams(
                    PanMotorParams.POWER_LIMIT, PanMotorParams.MIN_POS, PanMotorParams.MAX_POS));
            shooterParams.setPanMotorPosPresets(PanMotorParams.POS_PRESET_TOLERANCE, PanMotorParams.posPresets);
        }

        if (HAS_TILT_MOTOR)
        {
            shooterParams.setTiltMotor(
                TiltMotorParams.MOTOR_NAME, TiltMotorParams.MOTOR_TYPE, TiltMotorParams.MOTOR_INVERTED,
                TiltMotorParams.MOTOR_VOLTCOMP_ENABLED, TiltMotorParams.MOTOR_BRAKE_ENABLED, null, false, false,
                new TrcShooter.PanTiltParams(
                    TiltMotorParams.POWER_LIMIT, TiltMotorParams.MIN_POS, TiltMotorParams.MAX_POS));
            shooterParams.setTiltMotorPosPresets(TiltMotorParams.POS_PRESET_TOLERANCE, TiltMotorParams.posPresets);
        }

        shooter = new FtcShooter(SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(ShooterMotorParams.REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(ShooterMotorParams.motor1PidCoeffs)
                .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID), null);
        // For tuning shooter motor 1 PID.
        shooter1Velocity = new TrcDiscreteValue(
            SUBSYSTEM_NAME + ".motor1TargetVel",
            ShooterMotorParams.MIN_VEL, ShooterMotorParams.MAX_VEL,
            ShooterMotorParams.MIN_VEL_INC, ShooterMotorParams.MAX_VEL_INC,
            ShooterMotorParams.DEF_VEL, ShooterMotorParams.DEF_VEL_INC);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(ShooterMotorParams.REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(ShooterMotorParams.motor2PidCoeffs)
                    .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID), null);
            // For tuning shooter motor 2 PID.
            shooter2Velocity = new TrcDiscreteValue(
                SUBSYSTEM_NAME + ".motor2TargetVel",
                ShooterMotorParams.MIN_VEL, ShooterMotorParams.MAX_VEL,
                ShooterMotorParams.MIN_VEL_INC, ShooterMotorParams.MAX_VEL_INC,
                ShooterMotorParams.DEF_VEL, ShooterMotorParams.DEF_VEL_INC);
        }
        else
        {
            shooter2Velocity = null;
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(PanMotorParams.DEG_PER_COUNT, PanMotorParams.POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(PanMotorParams.pidCoeffs)
                    .setPidControlParams(PanMotorParams.PID_TOLERANCE, PanMotorParams.USE_SOFTWARE_PID),
                null);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                PanMotorParams.STALL_MIN_POWER, PanMotorParams.STALL_TOLERANCE, PanMotorParams.STALL_TIMEOUT,
                PanMotorParams.STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(PanMotorParams.MIN_POS, PanMotorParams.MAX_POS, false);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(TiltMotorParams.DEG_PER_COUNT, TiltMotorParams.POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(TiltMotorParams.pidCoeffs)
                    .setPidControlParams(TiltMotorParams.PID_TOLERANCE, TiltMotorParams.USE_SOFTWARE_PID),
                null);
            motor.setSoftPositionLimits(TiltMotorParams.MIN_POS, TiltMotorParams.MAX_POS, false);
        }

        if (HAS_LAUNCHER)
        {
            FtcServoActuator.Params launcherParams = new FtcServoActuator.Params()
                .setPrimaryServo(LauncherParams.SERVO_NAME, LauncherParams.SERVO_INVERTED);
            launcher = new FtcServoActuator(launcherParams).getServo();
        }
        else
        {
            launcher = null;
        }
    }   //Shooter

    /**
     * This method returns the created shooter.
     *
     * @return created shooter.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    /**
     * This method is called to launch the game piece into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     * @param context specifies the context object passed to the ShootOp method.
     */
    public void shoot(String owner, TrcEvent completionEvent, Object context)
    {
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        if (launcher != null)
        {
            robot.globalTracer.traceInfo(instanceName, "shoot(owner=" + owner + ", event=" + completionEvent + ")");
            launchOwner = owner;
            launchCompletionEvent = completionEvent;
            launchCallbackEvent = new TrcEvent(SUBSYSTEM_NAME + ".launchCallback");
            launchCallbackEvent.setCallback(this::launchCallback, null);
            launcher.setPosition(
                owner, 0.0, LauncherParams.LAUNCH_POS, launchCallbackEvent, LauncherParams.LAUNCH_DURATION);
        }
        else if (completionEvent != null)
        {
            robot.globalTracer.traceInfo(instanceName, "There is no launcher, signal completion anyway.");
            completionEvent.signal();
        }
    }   //shoot

    /**
     * This method is called when the launch duration has expired.
     *
     * @param context not used.
     * @param canceled specifies true if launch was canceled (not used).
     */
    private void launchCallback(Object context, boolean canceled)
    {
        // Reset launcher, fire and forget.
        launcher.setPosition(launchOwner, 0.0, LauncherParams.REST_POS, null, 0.0);
        if (launchCompletionEvent != null)
        {
            if (canceled)
            {
                launchCompletionEvent.cancel();
            }
            else
            {
                launchCompletionEvent.signal();
            }
            launchCompletionEvent = null;
        }
        launchOwner = null;
        launchCallbackEvent = null;
    }   //launchCallback

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        shooter.cancel();
        if (launcher != null)
        {
            launcher.cancel();
        }
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Shooter does not need zero calibration.
        // Tilter has absolute encoder and therefore no need for zero calibration.
        // Zero calibrate turret (pan).
        shooter.panMotor.zeroCalibrate(owner, PanMotorParams.ZERO_CAL_POWER, completionEvent, 0.0);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Shooter does not support resetState.
        // If you need to tuck away pan and tilt for turtle mode, add code here.
    }   //resetState

    /**
     * This method is called when gamepad analog control is operated on the subsystem.
     *
     * @param altFunc specifies true if the gamepad AltFunc button is pressed, false otherwise.
     * @param inputs specifies an array of analog values.
     */
    @Override
    public void subsystemControl(boolean altFunc, double... inputs)
    {
        double power = inputs[0];
        if (shooter.tiltMotor != null && power != prevTiltPower)
        {
            if (altFunc)
            {
                // Manual override.
                shooter.tiltMotor.setPower(power);
            }
            else
            {
                shooter.tiltMotor.setPidPower(
                    power, TiltMotorParams.POWER_LIMIT, TiltMotorParams.MIN_POS, TiltMotorParams.MAX_POS, true);
            }
            prevTiltPower = power;
        }

        power = inputs[1];
        if (shooter.panMotor != null && power != prevPanPower)
        {
            if (altFunc)
            {
                // Manual override.
                shooter.panMotor.setPower(power);
            }
            else
            {
                shooter.panMotor.setPidPower(
                    power, PanMotorParams.POWER_LIMIT, PanMotorParams.MIN_POS, PanMotorParams.MAX_POS, true);
            }
            prevPanPower = power;
        }
    }   //subsystemControl

    /**
     * This method is called to perform the subsystem action.
     *
     * @param action specifies the subsystem action to perform.
     * @param context specifies the context object for the action.
     */
    @Override
    public void subsystemAction(Action action, Object context)
    {
        switch (action)
        {
            case ToggleAutoShoot:
                if (robot.autoShootTask.isActive())
                {
                    robot.autoShootTask.cancel();
                    shooter.tracer.traceInfo(instanceName, ">>>>> Cancel Auto Shoot");
                }
                else
                {
                    boolean useVision = context != null && (Boolean) context;

                    robot.autoShootTask.autoShoot(instanceName, null, useVision, (int[]) null);
                    shooter.tracer.traceInfo(instanceName, ">>>>> Auto Shoot");
                }
                break;

            case ToggleManualShoot:
                if (shooter.isActive())
                {
                    shooter.cancel(instanceName);
                    shooter.tracer.traceInfo(instanceName, ">>>>> Cancel Manual Shoot");
                }
                else
                {
                    robot.shooter.aimShooter(
                        instanceName, robot.shooterSubsystem.shooter1Velocity.getValue(), 0.0, null, null, null, 0.0,
                        robot.shooterSubsystem::shoot, null, Shooter.ShooterMotorParams.OFF_DELAY);
                    shooter.tracer.traceInfo(instanceName, ">>>>> Manual Shoot");
                }
                break;

            case IncShooterVelocity:
                shooter1Velocity.upValue();
                Dashboard.TuneShootTable.shootMotor1Velocity = shooter1Velocity.getValue();
                shooter.tracer.traceInfo(instanceName, ">>>>> Shooter velocity up");
                break;

            case DecShooterVelocity:
                shooter1Velocity.downValue();
                Dashboard.TuneShootTable.shootMotor1Velocity = shooter1Velocity.getValue();
                shooter.tracer.traceInfo(instanceName, ">>>>> Shooter velocity down");
                break;

            case IncShooterVelIncrement:
                shooter1Velocity.upIncrement();
                shooter.tracer.traceInfo(instanceName, ">>>>> Shooter velocity increment up");
                break;

            case DecShooterVelIncrement:
                shooter1Velocity.downIncrement();
                shooter.tracer.traceInfo(instanceName, ">>>>> Shooter velocity increment down");
                break;

            default:
                break;
        }
    }   //subsystemAction

    /**
     * This method is called to perform subsystem tuning action.
     *
     * @param action specifies the subsystem tuning action to perform.
     * @param tuneSubsystemName specifies the subsystem object to tune.
     */
    @Override
    public void tuneSubsystem(TuneAction action, String tuneSubsystemName)
    {
        Double target = null;

        if (tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp?
                shooter1Velocity.upValue(): shooter1Velocity.downValue();
            shooter.setShooterMotorRPM(target, null);
        }
        else if (shooter.shooterMotor2 != null &&
                 tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp?
                shooter2Velocity.upValue(): shooter2Velocity.downValue();
            shooter.setShooterMotorRPM(null, target);
        }
        else if (shooter.panMotor != null &&
                 tuneSubsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp?
                shooter.panMotor.presetPositionUp(null, null): shooter.panMotor.presetPositionDown(null, null);
        }
        else if (shooter.tiltMotor != null &&
                 tuneSubsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp?
                shooter.tiltMotor.presetPositionUp(null, null): shooter.tiltMotor.presetPositionDown(null, null);
        }
        else if (launcher != null &&
                 tuneSubsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp? LauncherParams.LAUNCH_POS: LauncherParams.REST_POS;
            launcher.setPosition(target);
        }

        if (target != null)
        {
            Dashboard.TuneSubsystem.target = target;
            shooter.tracer.traceInfo(
                instanceName, "Tune %s %s: target=%.3f",
                tuneSubsystemName, action == TuneAction.SetNextTuneTargetUp? "Up": "Down", target);
        }
    }   //tuneSubsystem

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        // Not applicable for FTC.
    }   //publishToDashboard

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop)
        {
            TrcMotor motor;

            motor = shooter.getShooterMotor1();
            dashboard.displayPrintf(
                lineNum++, "%sMotor1: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
                SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(),
                shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM());

            motor = shooter.getShooterMotor2();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%sMotor2: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
                    SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(),
                    shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM());
            }

            motor = shooter.getPanMotor();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%sPanMotor: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                    SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(),
                    motor.getPidTarget());
            }

            motor = shooter.getTiltMotor();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%sTiltMotor: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                    SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(),
                    motor.getPidTarget());
            }

            if (launcher != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%sLauncher: pos=%.3f", SUBSYSTEM_NAME, launcher.getPosition());
            }
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null)
        {
            if (tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
            {
                Dashboard.TuneSubsystem.input = shooter.getShooterMotor1RPM();
            }
            else if (shooter.shooterMotor2 != null &&
                    tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
            {
                Dashboard.TuneSubsystem.input = shooter.getShooterMotor2RPM();
            }
            else if (shooter.panMotor != null && tuneSubsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
            {
                Dashboard.TuneSubsystem.input = shooter.getPanAngle();
            }
            else if (shooter.tiltMotor != null && tuneSubsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
            {
                Dashboard.TuneSubsystem.input = shooter.getTiltAngle();
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName)
    {
        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = ShooterMotorParams.motor1PidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = ShooterMotorParams.PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = ShooterMotorParams.USE_SOFTWARE_PID;
            Dashboard.TuneSubsystem.target = shooter1Velocity.getValue();
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = ShooterMotorParams.motor2PidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = ShooterMotorParams.PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = ShooterMotorParams.USE_SOFTWARE_PID;
            Dashboard.TuneSubsystem.target = shooter2Velocity.getValue();
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = PanMotorParams.pidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = PanMotorParams.PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = PanMotorParams.USE_SOFTWARE_PID;
            Dashboard.TuneSubsystem.target = PanMotorParams.FRONT_POS;
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = TiltMotorParams.pidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = TiltMotorParams.PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = TiltMotorParams.USE_SOFTWARE_PID;
            Dashboard.TuneSubsystem.target = TiltMotorParams.MIN_POS;
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            Dashboard.TuneSubsystem.target = LauncherParams.REST_POS;
        }
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsFromDashboard(String subsystemName)
    {
        TrcMotor.PidParams pidParams = new TrcMotor.PidParams()
            .setPidCoefficients(Dashboard.TuneSubsystem.pidCoeffs)
            .setPidControlParams(
                Dashboard.TuneSubsystem.pidTolerance, Dashboard.TuneSubsystem.useSoftwarePid);
        Double target = null;

        tuneSubsystemName = null;
        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            target = Dashboard.TuneSubsystem.target;
            shooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
            shooter1Velocity.setValue(target);
            shooter.setShooterMotorRPM(target, null);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            target = Dashboard.TuneSubsystem.target;
            shooter.shooterMotor2.setVelocityPidParameters(pidParams, null);
            shooter2Velocity.setValue(target);
            shooter.setShooterMotorRPM(null, target);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            target = Dashboard.TuneSubsystem.target;
            shooter.panMotor.setPositionPidParameters(pidParams, null);
            shooter.panMotor.setPosition(instanceName, 0.0, target, true, null, null, 0.0);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            target = Dashboard.TuneSubsystem.target;
            shooter.tiltMotor.setPositionPidParameters(pidParams, null);
            shooter.tiltMotor.setPosition(instanceName, 0.0, target, true, null, null, 0.0);
            tuneSubsystemName = subsystemName;
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            target = Dashboard.TuneSubsystem.target;
            launcher.setPosition(target);
            tuneSubsystemName = subsystemName;
            pidParams = null;
        }

        if (target != null)
        {
            // Matched a subsystem name.
            shooter.tracer.traceInfo(
                instanceName, "Tune %s: pidParams=%s, target=%.3f", subsystemName, pidParams, target);
        }
    }   //updateParamsFromDashboard

}   //class Shooter
