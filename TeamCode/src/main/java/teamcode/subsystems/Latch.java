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
import ftclib.motor.FtcServoActuator;
import teamcode.Dashboard;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Servo Latch Subsystem. The Latch subsystem is just a simple servo that moves between
 * two positions: latched and unlatched. It also supports analog control that allows using a joystick to speed
 * control the latch for tuning purpose.
 */
public class Latch extends TrcSubsystem<Latch.Action>
{
    public static final String SUBSYSTEM_NAME = "Latch";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        public static final String SERVO_NAME                   = SUBSYSTEM_NAME + ".servo";
        public static final boolean SERVO_INVERTED              = true;

        public static final double MAX_STEP_RATE                = 420.0;
        public static final double PHYSICAL_MIN_POS             = 0.0;
        public static final double PHYSICAL_MAX_POS             = 90.0;
        public static final double LOGICAL_MIN_POS              = 0.17;
        public static final double LOGICAL_MAX_POS              = 0.57;
    }   //class Params

    public enum Action
    {
        TogglePos,
        PresetPosUp,
        PresetPosDown
    }   //enum Action

    private final FtcDashboard dashboard;
    private final TrcServo servo;
    private double prevLatchPower = 0.0;
    private boolean latched = false;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Latch()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params latchParams = new FtcServoActuator.Params()
            .setPrimaryServo(Params.SERVO_NAME, Params.SERVO_INVERTED)
            .setPhysicalPosRange(Params.PHYSICAL_MIN_POS, Params.PHYSICAL_MAX_POS)
            .setLogicalPosRange(Params.LOGICAL_MIN_POS, Params.LOGICAL_MAX_POS)
            .setMaxStepRate(Params.MAX_STEP_RATE);

        servo = new FtcServoActuator(latchParams).getServo();
        servo.setPosition(Params.PHYSICAL_MIN_POS);
    }   //Latch

    /**
     * This method returns the created TrcServo.
     *
     * @return created servo.
     */
    public TrcServo getServo()
    {
        return servo;
    }   //getServo

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        servo.cancel();
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
        // Servos don't need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        servo.setPosition(Params.PHYSICAL_MAX_POS);
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
        if (power != prevLatchPower)
        {
            servo.setPower(power);
            prevLatchPower = power;
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
        if (action == Action.TogglePos)
        {
            latched = !latched;
            if (latched)
            {
                servo.setPosition(Params.PHYSICAL_MIN_POS);
            }
            else
            {
                servo.setPosition(Params.PHYSICAL_MAX_POS);
            }
            servo.tracer.traceInfo(instanceName, ">>>>> Toggle Latch: latched=" + latched);
        }
        else if (action == Action.PresetPosUp)
        {
            servo.presetPositionUp(null);
            servo.tracer.traceInfo(instanceName, ">>>>> Latch preset position up.");
        }
        else if (action == Action.PresetPosDown)
        {
            servo.presetPositionDown(null);
            servo.tracer.traceInfo(instanceName, ">>>>> Latch preset position down.");
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
        if (tuneSubsystemName.equalsIgnoreCase(Params.SERVO_NAME))
        {
            double target = action == TuneAction.SetNextTuneTargetUp? Params.LOGICAL_MAX_POS: Params.LOGICAL_MIN_POS;
            servo.setPosition(target);
            servo.tracer.traceInfo(instanceName, "Tune %s: target=%.3f", tuneSubsystemName, target);
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
                lineNum++, "%s: phyPos=%.3f, logPos=%.3f",
                SUBSYSTEM_NAME, servo.getPosition(), servo.getLogicalPosition());
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
        if (subsystemName.equalsIgnoreCase(Params.SERVO_NAME))
        {
            Dashboard.TuneSubsystem.target = Params.LOGICAL_MIN_POS;
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
        if (subsystemName.equalsIgnoreCase(Params.SERVO_NAME))
        {
            servo.setLogicalPosition(Dashboard.TuneSubsystem.target);
            servo.tracer.traceInfo(
                instanceName, "Tune %s: target=%.3f", subsystemName, Dashboard.TuneSubsystem.target);
        }
    }   //updateParamsFromDashboard

}   //class Latch
