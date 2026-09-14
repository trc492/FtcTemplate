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
 * This class implements an Elevator Subsystem. This implementation consists of one or two motors with built-in
 * encoders and optional limit switches for zero calibrating the relative encoder. If there is no lower limit
 * switch, it will use motor stall detection to zero calibrate the built-in relative encoder. It supports gravity
 * compensation. In the case of an elevator, gravity compensation power is the constant power required to hold the
 * elevator at any position.
 */
public class Elevator extends TrcSubsystem<Elevator.Action>
{
    public static final String SUBSYSTEM_NAME = "Elevator";
    private static final boolean NEED_ZERO_CAL = true;

    public static final class Params
    {
        private static final boolean HAS_TWO_MOTORS             = false;
        private static final boolean HAS_LOWER_LIMIT_SWITCH     = false;
        private static final boolean HAS_UPPER_LIMIT_SWITCH     = false;

        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;
        public static final boolean PRIMARY_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean PRIMARY_MOTOR_BRAKE_ENABLED = true;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;
        public static final boolean FOLLOWER_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean FOLLOWER_MOTOR_BRAKE_ENABLED= true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final String UPPER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".upperLimit";
        public static final boolean UPPER_LIMIT_SWITCH_INVERTED = false;

        public static final double POS_PID_TOLERANCE            = 0.1;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, 0.0);

        public static final double POS_OFFSET                   = 13.5;
        public static final double INCHES_PER_COUNT             = (32.125 - POS_OFFSET) / 5024.0;
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 32.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 20.0, 25.0, 30.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
        public static final double GRAVITY_COMP_POWER           = 0.0;
        public static final double ZERO_CAL_POWER               = -0.25;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;

        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
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
    private double prevElevatorPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED,
                Params.PRIMARY_MOTOR_VOLTCOMP_ENABLED, Params.PRIMARY_MOTOR_BRAKE_ENABLED)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);

        if (Params.HAS_TWO_MOTORS)
        {
            motorParams.addFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED,
                Params.FOLLOWER_MOTOR_VOLTCOMP_ENABLED, Params.FOLLOWER_MOTOR_BRAKE_ENABLED);
        }

        if (Params.HAS_LOWER_LIMIT_SWITCH)
        {
            motorParams.setLowerLimitSwitch(Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_INVERTED);
        }

        if (Params.HAS_UPPER_LIMIT_SWITCH)
        {
            motorParams.setUpperLimitSwitch(Params.UPPER_LIMIT_SWITCH_NAME, Params.UPPER_LIMIT_SWITCH_INVERTED);
        }

        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.USE_SOFTWARE_PID), null);
        motor.setPositionPidPowerComp(this::getGravityComp);

        if (!Params.HAS_LOWER_LIMIT_SWITCH)
        {
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
        }
    }   //Elevator

    /**
     * This method returns the created Elevator motor.
     *
     * @return created elevator motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method calculates the power required to make the elevator gravity neutral.
     *
     * @param motor specifies the motor for determining its gravity comp power.
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getGravityComp(TrcMotor motor, double currPower)
    {
        return tuneGravityCompPower != null? tuneGravityCompPower: Params.GRAVITY_COMP_POWER;
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
        motor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, completionEvent, Params.ZERO_CAL_TIMEOUT);
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

        if (power != prevElevatorPower)
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
            prevElevatorPower = power;
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
                motor.tracer.traceInfo(instanceName, ">>>>> Elevator preset position up.");
                break;

            case PresetPosDown:
                motor.presetPositionDown(null, Params.POWER_LIMIT);
                motor.tracer.traceInfo(instanceName, ">>>>> Elevator preset position down.");
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
                    lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f, LimitSw=%s/%s",
                    SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget(),
                    motor.isLowerLimitSwitchActive(), motor.isUpperLimitSwitchActive());
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

}   //class Elevator
