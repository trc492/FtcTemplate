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
    public static final String SUBSYSTEM_NAME = "DiffyServoWrist";
    private static final boolean NEED_ZERO_CAL = false;

    public static class Params
    {
        public static final String SERVO1_NAME                  = SUBSYSTEM_NAME + ".servo1";
        public static final boolean SERVO1_INVERTED             = false;

        public static final String SERVO2_NAME                  = SUBSYSTEM_NAME + ".servo2";
        public static final boolean SERVO2_INVERTED             = !SERVO1_INVERTED;

        public static final double MAX_STEP_RATE                = 300.0;    // deg/sec (max 520)
        public static final double LOGICAL_MIN_POS              = 0.15;
        public static final double LOGICAL_MAX_POS              = 0.85;
        public static final double PHYSICAL_POS_RANGE           = 230.0;
        public static final double TILT_POS_OFFSET              = -20.0;
        public static final double ROTATE_POS_OFFSET            = -1.0;

        public static final double TILT_MIN_POS                 = -90.0;
        public static final double TILT_MAX_POS                 = 90.0;
        public static final double ROTATE_MIN_POS               = -90.0;
        public static final double ROTATE_MAX_POS               = 90.0;

        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] tiltPosPresets             = {-110.0, -90.0, -45.0, 0.0, 45.0, 90.0, 110.0};
        public static final double[] rotatePosPresets           = {-90.0, -45.0, 0.0, 45.0, 90.0};
    }   //class Params

    private final FtcDashboard dashboard;
    public final TrcDifferentialServoWrist wrist;
    private double prevTiltPower = 0.0;
    private double prevRotatePower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public DiffyServoWrist()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcDifferentialServoWrist.Params wristParams = new FtcDifferentialServoWrist.Params()
            .setServos(Params.SERVO1_NAME, Params.SERVO1_INVERTED, Params.SERVO2_NAME, Params.SERVO2_INVERTED)
            .setPosRange(
                Params.LOGICAL_MIN_POS, Params.LOGICAL_MAX_POS, Params.PHYSICAL_POS_RANGE, Params.TILT_POS_OFFSET,
                Params.ROTATE_POS_OFFSET)
            .setMaxStepRate(Params.MAX_STEP_RATE)
            .setPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, Params.ROTATE_MIN_POS, Params.ROTATE_MAX_POS)
            .setPosPresets(Params.POS_PRESET_TOLERANCE, Params.tiltPosPresets, Params.rotatePosPresets);

        wrist = new FtcDifferentialServoWrist(SUBSYSTEM_NAME, wristParams).getWrist();
        wrist.setPosition(90.0, 0.0);
    }   //DiffyServoWrist

    /**
     * This method returns the wrist tilt position in degrees.
     *
     * @return wrist tilt position.
     */
    public double getTiltPosition()
    {
        return wrist.getTiltPosition();
    }   //getTiltPosition

    /**
     * This method returns the wrist rotate position in degrees.
     *
     * @return wrist rotate position.
     */
    public double getRotatePosition()
    {
        return wrist.getRotatePosition();
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
        wrist.setPosition(owner, delay, tiltPos, rotatePos, completionEvent, timeout);
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
        wrist.tiltPresetPositionUp(owner);
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
        wrist.tiltPresetPositionDown(owner);
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
        wrist.rotatePresetPositionUp(owner);
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
        wrist.rotatePresetPositionDown(owner);
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
        wrist.cancel();
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
        setPosition(-90.0, 0.0);
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
        double rotatePower = inputs[0];
        double tiltPower = inputs[1];

        if (rotatePower != prevRotatePower || tiltPower != prevTiltPower)
        {
            wrist.setPower(tiltPower, rotatePower);
            prevRotatePower = rotatePower;
            prevTiltPower = tiltPower;
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
                lineNum++, "%s: tilt(pwr/pos)=%.1f/%.1f,rotate(pwr/pos)=%.1f/%.1f",
                SUBSYSTEM_NAME, wrist.getTiltPower(), wrist.getTiltPosition(), wrist.getRotatePower(),
                wrist.getRotatePosition());
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
        // DiffyWirst doesn't support tuning.
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
        // DiffyWirst doesn't support tuning.
    }   //updateParamsFromDashboard

    /**
     * This method is called to set the next tune target up from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetUp(String subsystemName)
    {
        // DiffyWirst doesn't support tuning.
    }   //setNextTuneTargetUp

    /**
     * This method is called to set the next tune target down from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetDown(String subsystemName)
    {
        // DiffyWirst doesn't support tuning.
    }   //setNextTuneTargetDown

}   //class DiffyServoWrist
