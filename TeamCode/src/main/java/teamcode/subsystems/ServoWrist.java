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
 * This class creates the Servo Wrist subsystem. This implementation is a 1-DOF system that consists of a servo.
 * It allows the wrist to tilt up and down. Angular servos have a limited range of movement. Therefore, it limits
 * the tilting range of the wrist.
 */
public class ServoWrist extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "ServoWrist";
    private static final boolean NEED_ZERO_CAL = false;

    public static class Params
    {
        public static final String SERVO_NAME                   = SUBSYSTEM_NAME + ".servo";
        public static final boolean SERVO_INVERTED              = false;

        public static final double LOGICAL_MIN_POS              = 0.1;
        public static final double LOGICAL_MAX_POS              = 0.8;
        public static final double PHYSICAL_MIN_POS             = -90.0;    // in degrees
        public static final double PHYSICAL_MAX_POS             = 90.0;     // in degrees

        public static final double POS_PRESET_TOLERANCE         = 1.0;      // in degrees
        public static final double[] posPresets                 = {-110.0, -90.0, -45.0, 0.0, 45.0, 90.0, 110.0};
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcServo servo;
    private double prevWristPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoWrist()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
            .setPrimaryServo(Params.SERVO_NAME, Params.SERVO_INVERTED)
            .setLogicalPosRange(Params.LOGICAL_MIN_POS, Params.LOGICAL_MAX_POS)
            .setPhysicalPosRange(Params.PHYSICAL_MIN_POS, Params.PHYSICAL_MAX_POS)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);

        servo = new FtcServoActuator(wristParams).getServo();
    }   //ServoWrist

    /**
     * This method returns the created servo.
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
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        servo.setPosition(-90.0);
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

        if (power != prevWristPower)
        {
            servo.setPower(power);
            prevWristPower = power;
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
                lineNum++, "%s: pos=%.3f/%.3f", SUBSYSTEM_NAME, servo.getPosition(), servo.getLogicalPosition());
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

    /**
     * This method is called to set the next tune target up from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetUp(String subsystemName)
    {
        if (subsystemName.equalsIgnoreCase(Params.SERVO_NAME))
        {
            double target = Params.LOGICAL_MAX_POS;
            servo.setLogicalPosition(target);
            servo.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, target);
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
        if (subsystemName.equalsIgnoreCase(Params.SERVO_NAME))
        {
            double target = Params.LOGICAL_MIN_POS;
            servo.setLogicalPosition(target);
            servo.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class ServoWrist
