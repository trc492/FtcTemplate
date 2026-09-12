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
 * This class implements a Turret Subsystem. This implementation consists of a motor with built-in encoder. It has
 * a lower limit switch for zero calibrating the built-in relative encoder. Since Turret is circular in nature, it
 * is recommended to implement a hard stop to prevent the Turret from overrunning the upper limit causing the wiring
 * harness to be twisted. Even though we do implement soft limits on the Turret, hard stop would prevent folks from
 * spinning the turret round and round twisting the wiring harness when the robot is off.
 */
public class Turret extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Turret";
    private static final boolean NEED_ZERO_CAL = true;

    public static final class Params
    {
        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final boolean MOTOR_INVERTED              = true;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.04, 0.0, 0.0, 0.0, 0.0);

        public static final double ENCODER_PPR                  = 288.0;
        public static final double GEAR_RATIO                   = 100.0/60.0;
        public static final double DEG_PER_COUNT                = 360.0/(ENCODER_PPR*GEAR_RATIO);
        public static final double POS_OFFSET                   = 0.0;
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 325.0;
        public static final double BACK_POS                     = 0.0;
        public static final double LEFT_POS                     = 90.0;
        public static final double FRONT_POS                    = 180.0;
        public static final double RIGHT_POS                    = 270.0;
        public static final double TURTLE_POS                   = FRONT_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {BACK_POS, LEFT_POS, FRONT_POS, RIGHT_POS};

        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.3;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor motor;
    private String tuneSubsystemName = null;
    private double prevTurretPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Turret()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED, Params.MOTOR_VOLTCOMP_ENABLED,
                Params.MOTOR_BRAKE_ENABLED)
            .setLowerLimitSwitch(Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.USE_SOFTWARE_PID), null);
        // Since we don't have upper limit switch, setting soft limits will protect turret from overrunning the upper
        // limit in manual mode.
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //Turret

    /**
     * This method returns the created motor.
     *
     * @return created motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

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
        if (power != prevTurretPower)
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
            prevTurretPower = power;
        }
    }   //subsystemControl

    /**
     * This method is called when a gamepad button is pressed to perform the subsystem action.
     *
     * @param pressed specifies true if the gamepad button is pressed, false otherwise.
     * @param altFunc specifies true if the gamepad AltFunc button is pressed, false otherwise.
     */
    @Override
    public void subsystemAction(boolean pressed, boolean altFunc)
    {
    }   //subsystemAction

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
                lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f, lowerLimit=%s",
                SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget(),
                motor.isLowerLimitSwitchActive());
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null && tuneSubsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
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
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            Dashboard.TuneSubsystem.pidCoeffs = Params.posPidCoeffs;
            Dashboard.TuneSubsystem.pidTolerance = Params.POS_PID_TOLERANCE;
            Dashboard.TuneSubsystem.useSoftwarePid = Params.USE_SOFTWARE_PID;
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
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            TrcMotor.PidParams pidParams = new TrcMotor.PidParams()
                .setPidCoefficients(Dashboard.TuneSubsystem.pidCoeffs)
                .setPidControlParams(
                    Dashboard.TuneSubsystem.pidTolerance, Dashboard.TuneSubsystem.useSoftwarePid);
            motor.setPositionPidParameters(pidParams, null);
            motor.setPosition(Dashboard.TuneSubsystem.target);
            motor.tracer.traceInfo(
                instanceName, "Tune %s: PidParams=%s, target=%.3f",
                subsystemName, pidParams, Dashboard.TuneSubsystem.target);
            tuneSubsystemName = subsystemName;
        }
    }   //updateParamsFromDashboard

    /**
     * This method is called to set the next tune target up from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetUp(String subsystemName)
    {
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            double target = motor.presetPositionUp(null, null);
            motor.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetUp

    /**
     * This method is called to set the next tune target down from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetDown(String subsystemName)
    {
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            double target = motor.presetPositionDown(null, null);
            motor.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class Turret
