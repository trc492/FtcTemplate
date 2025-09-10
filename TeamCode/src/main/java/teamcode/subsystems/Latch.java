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
 * This class implements a Servo Latch Subsystem. The Latch subsystem is just a simple servo that moves between
 * two positions: latched and unlatched. It also supports analog control that allows using a joystick to speed
 * control the latch for tuning purpose.
 */
public class Latch extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Latch";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String SERVO_NAME                   = SUBSYSTEM_NAME + ".servo";
        public static final boolean SERVO_INVERTED              = true;

        public static final double PHYSICAL_MIN_POS             = 0.0;
        public static final double PHYSICAL_MAX_POS             = 90.0;
        public static final double LOGICAL_MIN_POS              = 0.17;
        public static final double LOGICAL_MAX_POS              = 0.57;
        public static final double MAX_STEP_RATE                = 420.0;
        public static final double[] posPresets                 = {PHYSICAL_MIN_POS, 30.0, 60.0, PHYSICAL_MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 1.0;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcServo latch;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Latch()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params latchParams = new FtcServoActuator.Params()
            .setPrimaryServo(Params.SERVO_NAME, Params.SERVO_INVERTED)
            .setPhysicalPosRange(Params.PHYSICAL_MIN_POS, Params.PHYSICAL_MAX_POS)
            .setLogicalPosRange(Params.LOGICAL_MIN_POS, Params.LOGICAL_MAX_POS)
            .setMaxStepRate(Params.MAX_STEP_RATE)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);

        latch = new FtcServoActuator(latchParams).getServo();
        latch.setPosition(Params.PHYSICAL_MIN_POS);
    }   //Latch

    /**
     * This method returns the created TrcServo.
     *
     * @return created servo.
     */
    public TrcServo getServo()
    {
        return latch;
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
        latch.cancel();
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
        // Servos don't need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        latch.setPosition(Params.PHYSICAL_MAX_POS);
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
                lineNum++, "%s: phyPos=%.3f, logPos=%.3f",
                Params.SUBSYSTEM_NAME, latch.getPosition(), latch.getLogicalPosition());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - Logical position min
     *        tuneParam1 - Logical position max
     */
    @Override
    public void prepSubsystemForTuning(String subComponent, double... tuneParams)
    {
        latch.setLogicalPosRange(tuneParams[0], tuneParams[1]);
    }   //prepSubsystemForTuning

}   //class Latch
