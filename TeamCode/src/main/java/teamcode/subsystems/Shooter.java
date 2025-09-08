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
import ftclib.subsystem.FtcShooter;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcDiscreteValue;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcShootParamTable;
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
public class Shooter extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = false;
        public static final boolean HAS_PAN_MOTOR               = false;
        public static final boolean HAS_TILT_MOTOR              = false;

        // Shooter Motor1
        public static final String SHOOTER_MOTOR1_NAME          = SUBSYSTEM_NAME + ".shooterMotor1";
        public static final MotorType SHOOTER_MOTOR1_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR1_INVERTED     = true;

        // Shooter Motor2
        public static final String SHOOTER_MOTOR2_NAME          = SUBSYSTEM_NAME + ".shooterMotor2";
        public static final MotorType SHOOTER_MOTOR2_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR2_INVERTED     = true;

        // Assume shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final double GOBILDA6000_CPR              = 28.0;
        public static final double SHOOTER_GEAR_RATIO           = 24.0/36.0;
        public static final double SHOOTER_REV_PER_COUNT        = 1.0/(GOBILDA6000_CPR * SHOOTER_GEAR_RATIO);
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= true;
        public static final TrcPidController.PidCoefficients shooter1PidCoeffs =
            new TrcPidController.PidCoefficients(0.075, 0.0, 0.0, 0.008, 0.0);
        public static final TrcPidController.PidCoefficients shooter2PidCoeffs =
            new TrcPidController.PidCoefficients(0.075, 0.0, 0.0, 0.008, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 10.0/60.0;// in RPS (10 RPM)

        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 7360.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 10.0;     // in RPM
        public static final double SHOOTER_OFF_DELAY            = 0.5;      // in sec

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".panMotor";
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.DcMotor;
        public static final boolean PAN_MOTOR_INVERTED          = false;

        public static final double PAN_DEG_PER_COUNT            = 1.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;
        public static final TrcPidController.PidCoefficients panPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double PAN_PID_TOLERANCE            = 1.0;

        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -90.0;
        public static final double PAN_MAX_POS                  = 90.0;

        // Tilt Motor
        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".tiltMotor";
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.DcMotor;
        public static final boolean TILT_MOTOR_INVERTED         = false;

        public static final double TILT_DEG_PER_COUNT           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;
        public static final TrcPidController.PidCoefficients tiltPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double TILT_PID_TOLERANCE           = 1.0;

        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = 0.0;
        public static final double TILT_MAX_POS                 = 90.0;

        public static final TrcShootParamTable shootParamTable = new TrcShootParamTable()
            .add("test3ft", 36.0, 60.0, 0.0, 60.0)
            .add("test4ft", 48.0, 70.0, 0.0, 60.0)
            .add("test5ft", 60.0, 80.0, 0.0, 60.0)
            .add("test6ft", 72.0, 90.0, 0.0, 60.0);
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcRollerIntake intake;
    private final TrcShooter shooter;
    public final TrcDiscreteValue shooter1Velocity;
    public final TrcDiscreteValue shooter2Velocity;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter(TrcRollerIntake intake)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        this.intake = intake;

        FtcShooter.Params shooterParams = new FtcShooter.Params()
            .setShooterMotor1(
                Params.SHOOTER_MOTOR1_NAME, Params.SHOOTER_MOTOR1_TYPE, Params.SHOOTER_MOTOR1_INVERTED);

        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                Params.SHOOTER_MOTOR2_NAME, Params.SHOOTER_MOTOR2_TYPE, Params.SHOOTER_MOTOR2_INVERTED);
        }

        if (Params.HAS_PAN_MOTOR)
        {
            shooterParams.setPanMotor(
                Params.PAN_MOTOR_NAME, Params.PAN_MOTOR_TYPE, Params.PAN_MOTOR_INVERTED,
                new TrcShooter.PanTiltParams(Params.PAN_POWER_LIMIT, Params.PAN_MIN_POS, Params.PAN_MAX_POS));
        }

        if (Params.HAS_TILT_MOTOR)
        {
            shooterParams.setTiltMotor(
                Params.TILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.TILT_MOTOR_INVERTED,
                new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS));
        }

        shooter = new FtcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(Params.SHOOTER_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(
            Params.shooter1PidCoeffs, Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED);
        shooter1Velocity = new TrcDiscreteValue(
            Params.SUBSYSTEM_NAME + ".motor1TargetVel",
            Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
            Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
            Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(Params.SHOOTER_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                Params.shooter2PidCoeffs, Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED);
            shooter2Velocity = new TrcDiscreteValue(
                Params.SUBSYSTEM_NAME + ".motor2TargetVel",
                Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
                Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
                Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);
        }
        else
        {
            shooter2Velocity = null;
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(Params.PAN_DEG_PER_COUNT, 0.0);
            motor.setPositionPidParameters(
                Params.panPidCoeffs, Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(Params.TILT_DEG_PER_COUNT, 0.0);
            motor.setPositionPidParameters(
                Params.tiltPidCoeffs, Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED);
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
     * This method is called to feed the game piece to the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     */
    public void shoot(String owner, TrcEvent completionEvent)
    {
        if (intake != null)
        {
            intake.autoEject(owner, completionEvent, 0.0);
            TrcDbgTrace.globalTraceInfo(instanceName, "Shooter ready, initiate shoot.");
        }
        else if (completionEvent != null)
        {
            completionEvent.signal();
            TrcDbgTrace.globalTraceInfo(instanceName, "There is no intake, signal completion anyway.");
        }
    }   //shoot

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
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // Shooter does not need zero calibration.
        // Assuming pan and tilt are using absolute encoders. If not, we need to add code to zero calibrate pan and
        // tilt here.
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
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum)
    {
        TrcMotor motor;

        motor = shooter.getShooterMotor1();
        dashboard.displayPrintf(
            lineNum++, "%sMotor1: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
            Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(),
            shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM());

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            dashboard.displayPrintf(
                lineNum++, "%sMotor2: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
                Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(),
                shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM());
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            dashboard.displayPrintf(
                lineNum++, "%sPanMotor: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            dashboard.displayPrintf(
                lineNum++, "%sTiltMotor: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - Kp
     *        tuneParam1 - Ki
     *        tuneParam2 - Kd
     *        tuneParam3 - Kf
     *        tuneParam4 - iZone
     *        tuneParam5 - PidTolerance
     *        tuneParam6 - Shooter motor target velocity
     */
    @Override
    public void prepSubsystemForTuning(double... tuneParams)
    {
        // To tune Shooter Motor PID, uncomment the code below.
        shooter.getShooterMotor1().setVelocityPidParameters(
            tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
            Params.SHOOTER_SOFTWARE_PID_ENABLED);
        shooter1Velocity.setValue(tuneParams[6]);

        // To tune Shooter Motor PID, uncomment the code below.
        // shooter.getShooterMotor2().setVelocityPidParameters(
        //     tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
        //     Params.SHOOTER_SOFTWARE_PID_ENABLED);
        // shooter2Velocity.setValue(tuneParams[6]);

        // To tune Pan Motor PID, uncomment the code below.
        // shooter.getPanMotor().setPositionPidParameters(
        //     tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
        //     Params.PAN_SOFTWARE_PID_ENABLED);

        // To tune Tilt Motor PID, uncomment the code below.
        // shooter.getTiltMotor().setPositionPidParameters(
        //     tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
        //     Params.TILT_SOFTWARE_PID_ENABLED);
    }   //prepSubsystemForTuning

}   //class Shooter
