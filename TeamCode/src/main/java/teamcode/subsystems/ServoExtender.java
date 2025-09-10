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
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the Servo Extender subsystem. This implementation is a linear extender driven by two servos
 * to either extend or retract the extender.
 */
public class ServoExtender extends TrcSubsystem
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "ServoExtender";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String PRIMARY_SERVO_NAME           = Params.SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = Params.SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = false;

        public static double POS_RETRACT                        = 0.1;
        public static double POS_EXTEND                         = 0.8;
    }   //class Params

    private final FtcDashboard dashboard;
    public final TrcServo servo;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoExtender()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params extenderParams = new FtcServoActuator.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED);

        servo = new FtcServoActuator(extenderParams).getServo();
    }   //ServoExtender

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
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
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
                lineNum++, "%s: pos=%.3f, extended=%s", Params.SUBSYSTEM_NAME, servo.getPosition(), isExtended());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - retract position
     *        tuneParam1 - extend position
     */
    @Override
    public void prepSubsystemForTuning(String subComponent, double... tuneParams)
    {
        Params.POS_RETRACT = tuneParams[0];
        Params.POS_EXTEND = tuneParams[1];
    }   //prepSubsystemForTuning

}   //class ServoExtender
