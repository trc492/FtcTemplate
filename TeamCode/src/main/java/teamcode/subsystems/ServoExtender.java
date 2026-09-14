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
 * This class creates the Servo Extender subsystem. This implementation is a linear extender driven by two servos
 * to either extend or retract the extender.
 */
public class ServoExtender extends TrcSubsystem<ServoExtender.Action>
{
    public static final String SUBSYSTEM_NAME = "ServoExtender";
    private static final boolean NEED_ZERO_CAL = false;

    public static class Params
    {
        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = false;

        public static double POS_RETRACT                        = 0.1;
        public static double POS_EXTEND                         = 0.8;
    }   //class Params

    public enum Action
    {
        TogglePos,
        PresetPosUp,
        PresetPosDown
    }   //enum Action

    private final FtcDashboard dashboard;
    private final TrcServo servo;
    private double prevExtenderPower = 0.0;
    private boolean extended = false;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoExtender()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params extenderParams = new FtcServoActuator.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED);

        servo = new FtcServoActuator(extenderParams).getServo();
    }   //ServoExtender

    /**
     * This method returns the created servo.
     *
     * @return created servo.
     */
    public TrcServo getServo()
    {
        return servo;
    }   //getServo

    /**
     * This method checks if the extender is extended.
     *
     * @return true if extended, false otherwise.
     */
    public boolean isExtended()
    {
        return servo.getPosition() == Params.POS_EXTEND;
    }   //isExtended

    /**
     * This method sets the extender to extended position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(owner, delay, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, delay, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     *
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, 0.0, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     */
    public void extend()
    {
        servo.setPosition(null, 0.0, Params.POS_EXTEND, null, 0.0);
    }   //extend

    /**
     * This method sets the extender to retracted position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(owner, delay, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, delay, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     *
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, 0.0, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     */
    public void retract()
    {
        servo.setPosition(null, 0.0, Params.POS_RETRACT, null, 0.0);
    }   //retract

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
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        servo.setPosition(Params.POS_RETRACT);
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

        if (power != prevExtenderPower)
        {
            servo.setPower(power);
            prevExtenderPower = power;
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
            extended = !extended;
            if (extended)
            {
                extend();
            }
            else
            {
                retract();
            }
            servo.tracer.traceInfo(instanceName, ">>>>> Toggle Extender: extend=" + extended);
        }
        else if (action == Action.PresetPosUp)
        {
            servo.presetPositionUp(null);
            servo.tracer.traceInfo(instanceName, ">>>>> ServoExtender preset position up.");
        }
        else if (action == Action.PresetPosDown)
        {
            servo.presetPositionDown(null);
            servo.tracer.traceInfo(instanceName, ">>>>> ServoExtender preset position down.");
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
        if (tuneSubsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            double target = action == TuneAction.SetNextTuneTargetUp? Params.POS_EXTEND: Params.POS_RETRACT;
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
                lineNum++, "%s: pos=%.3f, extended=%s", SUBSYSTEM_NAME, servo.getPosition(), isExtended());
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            Dashboard.TuneSubsystem.target = Params.POS_RETRACT;
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            servo.setPosition(Dashboard.TuneSubsystem.target);
            servo.tracer.traceInfo(
                instanceName, "Tune %s: target=%.3f", subsystemName, Dashboard.TuneSubsystem.target);
        }
    }   //updateParamsFromDashboard

}   //class ServoExtender
