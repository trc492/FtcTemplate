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
import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcMotorActuator.MotorType;
import teamcode.Dashboard;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a CrServoArm Subsystem. This implementation consists of two Axon servos running in Continuous
 * Rotation mode with an analog absolute encoder. It does not require zero calibration. Therefore, limit switches are
 * optional. If using limit switches, they are for movement range protection. If not using limit switches, software
 * limit must be set. It supports gravity compensation by computing the power required to hold the arm at its current
 * angle.
 */
public class CrServoArm extends TrcSubsystem<CrServoArm.Action>
{
    public static final String SUBSYSTEM_NAME = "CrServoArm";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        public static final MotorType MOTOR_TYPE                = MotorType.CRServo;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final boolean PRIMARY_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean PRIMARY_MOTOR_BRAKE_ENABLED = false;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;
        public static final boolean FOLLOWER_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean FOLLOWER_MOTOR_BRAKE_ENABLED = false;

        public static final String ABSENC_NAME                  = SUBSYSTEM_NAME + ".absEnc";
        public static final boolean ABSENC_INVERTED             = true;

        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.0162, 0.0, 0.0, 0.0, 2.0);

        public static final double POS_DEG_SCALE                = 360.0;
        public static final double POS_OFFSET                   = 27.0;
        public static final double ABSENC_ZERO_OFFSET           = 0.949697;
        public static final double MIN_POS                      = 27.3;
        public static final double MAX_POS                      = 300.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double POS_PRESET_TOLERANCE         = 5.0;
        public static final double[] posPresets                 =
            {30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};

        public static final double POWER_LIMIT                  = 0.25;
        public static final double GRAVITY_COMP_POWER           = 0.1675;
    }   //class Params

    public enum Action
    {
        PresetPosUp,
        PresetPosDown
    }   //enum Action

    private final FtcDashboard dashboard;
    private final TrcMotor motor;
    private String tuneSubsystemName = null;
    private Double tuneGravityCompPower = null;
    private double prevArmPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CrServoArm()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED,
                Params.PRIMARY_MOTOR_VOLTCOMP_ENABLED, Params.PRIMARY_MOTOR_BRAKE_ENABLED)
            .addFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED,
                Params.FOLLOWER_MOTOR_VOLTCOMP_ENABLED, Params.FOLLOWER_MOTOR_BRAKE_ENABLED)
            .setExternalEncoder(Params.ABSENC_NAME, Params.ABSENC_INVERTED)
            .setPositionScaleAndOffset(Params.POS_DEG_SCALE, Params.POS_OFFSET, Params.ABSENC_ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.USE_SOFTWARE_PID), null);
        motor.setPositionPidPowerComp(this::getGravityComp);
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //CrServoArm

    /**
     * This method returns the created CrServoArm motor.
     *
     * @return created arm motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method calculates the power required to make the arm gravity neutral.
     *
     * @param motor specifies the motor for determining its gravity comp power.
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneGravityCompPower != null? tuneGravityCompPower: Params.GRAVITY_COMP_POWER;
        return gravityCompPower * Math.sin(Math.toRadians(motor.getPosition()));
    }   //getGravityComp

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        motor.cancel();
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
        // No zero calibration needed for absolute encoder.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        motor.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
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

        if (power != prevArmPower)
        {
            if (altFunc)
            {
                // Manual override.
                motor.setPower(power);
            }
            else
            {
                motor.setPidPower(power, Params.POWER_LIMIT, Params.MIN_POS, Params.MAX_POS, true);
            }
            prevArmPower = power;
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
            case PresetPosUp:
                motor.presetPositionUp(null, Params.POWER_LIMIT);
                motor.tracer.traceInfo(instanceName, ">>>>> CrServoArm preset position up.");
                break;

            case PresetPosDown:
                motor.presetPositionDown(null, Params.POWER_LIMIT);
                motor.tracer.traceInfo(instanceName, ">>>>> CrServoArm preset position down.");
                break;

            default:
                break;
        }
    }   //subsystemAction

    /**
     * This method is called to perform the subsystem tune action.
     *
     * @param action specifies the subsystem tune action to perform.
     * @param tuneSubsystemName specifies the subsystem object to tune.
     */
    @Override
    public void tuneSubsystem(TuneAction action, String tuneSubsystemName)
    {
        Double target = null;

        if (tuneSubsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            target = action == TuneAction.SetNextTuneTargetUp?
                motor.presetPositionUp(null, null): motor.presetPositionDown(null, null);
        }

        if (target != null)
        {
            Dashboard.TuneSubsystem.target = target;
            motor.tracer.traceInfo(
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
            dashboard.displayPrintf(
                lineNum++, "%s: power=%.3f, pos=%.3f/%.3f",
                SUBSYSTEM_NAME, motor.getPower(), motor.getPosition(), motor.getPidTarget());
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null && tuneSubsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            Dashboard.TuneSubsystem.input = motor.getPosition();
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = Params.posPidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = Params.POS_PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = Params.USE_SOFTWARE_PID;
            Dashboard.TuneSubsystem.gravityPower = Params.GRAVITY_COMP_POWER;
            Dashboard.TuneSubsystem.target = Params.MIN_POS;
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
        tuneSubsystemName = null;
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            TrcMotor.PidParams pidParams = new TrcMotor.PidParams()
                .setPidCoefficients(Dashboard.TuneSubsystem.pidCoeffs)
                .setPidControlParams(
                    Dashboard.TuneSubsystem.pidTolerance, Dashboard.TuneSubsystem.useSoftwarePid);

            tuneGravityCompPower = Dashboard.TuneSubsystem.gravityPower;
            motor.setPositionPidParameters(pidParams, null);
            motor.setPosition(Dashboard.TuneSubsystem.target);
            motor.tracer.traceInfo(
                instanceName, "Tune %s: PidParams=%s, target=%.3f, GravityPower=%.3f",
                subsystemName, pidParams, Dashboard.TuneSubsystem.target, tuneGravityCompPower);
            tuneSubsystemName = subsystemName;
        }
    }   //updateParamsFromDashboard

}   //class CrServoArm
