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
import ftclib.subsystem.FtcDifferentialServoWrist;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcDifferentialServoWrist;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the Differential Servo Wrist subsystem. This implementation is a 2-DOF system that consists of
 * two servos. When the two servos rotate in the same direction, the wrist will tilt up and down. When the two servos
 * rotate in opposite direction, the wrist will rotate. Regular servos have a limited range of movement. Because of
 * this, the tilt and rotation of the wrist will limit each other's range of motion. For example, if the wrist is
 * tilted to one extreme end, the wrist cannot rotate. If the wrist is in the middle tilt position, it will have
 * maximum rotation range and vice versa.
 */
public class DiffyServoWrist extends TrcSubsystem
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "DiffyServoWrist";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String SERVO1_NAME                  = Params.SUBSYSTEM_NAME + ".servo1";
        public static final boolean SERVO1_INVERTED             = false;
        public static final String SERVO2_NAME                  = Params.SUBSYSTEM_NAME + ".servo2";
        public static final boolean SERVO2_INVERTED             = !SERVO1_INVERTED;

        public static final double LOGICAL_MIN_POS              = 0.15;
        public static final double LOGICAL_MAX_POS              = 0.85;
        public static final double PHYSICAL_POS_RANGE           = 230.0;
        public static final double TILT_POS_OFFSET              = -20.0;
        public static final double ROTATE_POS_OFFSET            = -1.0;
        public static final double MAX_STEP_RATE                = 300.0;    // deg/sec (max 520)

        public static final double TILT_MIN_POS                 = -90.0;
        public static final double TILT_MAX_POS                 = 90.0;
        public static final double ROTATE_MIN_POS               = -90.0;
        public static final double ROTATE_MAX_POS               = 90.0;

        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] tiltPosPresets             = {-110, -90.0, -45.0, 0.0, 45.0, 90.0, 110};
        public static final double[] rotatePosPresets           = {-90.0, -45.0, 0.0, 45.0, 90.0};
    }   //class Params

    private final FtcDashboard dashboard;
    public final TrcDifferentialServoWrist diffyWrist;

    /**
     * Constructor: Creates an instance of the object.
     */
    public DiffyServoWrist()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcDifferentialServoWrist.Params wristParams = new FtcDifferentialServoWrist.Params()
            .setServos(Params.SERVO1_NAME, Params.SERVO1_INVERTED, Params.SERVO2_NAME, Params.SERVO2_INVERTED)
            .setPosRange(
                Params.LOGICAL_MIN_POS, Params.LOGICAL_MAX_POS, Params.PHYSICAL_POS_RANGE, Params.TILT_POS_OFFSET,
                Params.ROTATE_POS_OFFSET)
            .setMaxStepRate(Params.MAX_STEP_RATE)
            .setPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, Params.ROTATE_MIN_POS, Params.ROTATE_MAX_POS)
            .setPosPresets(Params.POS_PRESET_TOLERANCE, Params.tiltPosPresets, Params.rotatePosPresets);

        diffyWrist = new FtcDifferentialServoWrist(Params.SUBSYSTEM_NAME, wristParams).getWrist();
        diffyWrist.setPosition(90.0, 0.0);
    }   //DiffyServoWrist

    /**
     * This method returns the wrist tilt position in degrees.
     *
     * @return wrist tilt position.
     */
    public double getTiltPosition()
    {
        return diffyWrist.getTiltPosition();
    }   //getTiltPosition

    /**
     * This method returns the wrist rotate position in degrees.
     *
     * @return wrist rotate position.
     */
    public double getRotatePosition()
    {
        return diffyWrist.getRotatePosition();
    }   //getRotatePosition

    /**
     * This method sets the wrist position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided for no
     *        change.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(
        String owner, double delay, double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        if (rotatePos != null)
        {
            rotatePos %= 180.0;
            if (rotatePos > 90.0)
            {
                rotatePos -= 180.0;
            }
            else if (rotatePos < -90.0)
            {
                rotatePos += 180.0;
            }
        }
        else
        {
            rotatePos = getRotatePosition();
        }
        diffyWrist.setPosition(owner, delay, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided for no
     *        change.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double delay, double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, delay, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided for no
     *        change.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided for no
     *        change.
     */
    public void setPosition(double tiltPos, Double rotatePos)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, null, 0.0);
    }   //setPosition

    /**
     * This method sets the wrist to the next tilt preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionUp(String owner)
    {
        diffyWrist.tiltPresetPositionUp(owner);
    }   //tiltPresetPositionUp

    /**
     * This method sets the wrist to the next tilt preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionDown(String owner)
    {
        diffyWrist.tiltPresetPositionDown(owner);
    }   //tiltPresetPositionDown

    /**
     * This method sets the wrist to the next rotate preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionUp(String owner)
    {
        diffyWrist.rotatePresetPositionUp(owner);
    }   //rotatePresetPositionUp

    /**
     * This method sets the wrist to the next rotate preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionDown(String owner)
    {
        diffyWrist.rotatePresetPositionDown(owner);
    }   //rotatePresetPositionDown

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        diffyWrist.cancel();
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
        setPosition(-90.0, 0.0);
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
                lineNum++, "%s: tilt(pwr/pos)=%.1f/%.1f,rotate(pwr/pos)=%.1f/%.1f",
                Params.SUBSYSTEM_NAME, diffyWrist.getTiltPower(), diffyWrist.getTiltPosition(),
                diffyWrist.getRotatePower(), diffyWrist.getRotatePosition());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param subComponent specifies the sub-component of the Subsystem to be tuned, can be null if no sub-component.
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - Kp
     *        tuneParam1 - Ki
     *        tuneParam2 - Kd
     *        tuneParam3 - Kf
     *        tuneParam4 - iZone
     *        tuneParam5 - PidTolerance
     *        tuneParam6 - GravityCompPower
     */
    @Override
    public void prepSubsystemForTuning(String subComponent, double... tuneParams)
    {
        // DiffyWirst doesn't support tuning.
    }   //prepSubsystemForTuning

}   //class DiffyServoWrist
