/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Arm Subsystem. This implementation consists of a motor with built-in encoder. It does not
 * have any limit switches, so it is using motor stall detection to zero calibrate the built-in relative encoder. It
 * supports gravity compensation by computing the power required to hold the arm at its current angle.
 * There are many possible implementations by setting different parameters.
 * Please refer to the TrcLib documentation (<a href="https://trc492.github.io">...</a>) for details.
 */
public class Arm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Arm";
        public static final boolean NEED_ZERO_CAL               = true;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final FtcMotorActuator.MotorType MOTOR_TYPE= FtcMotorActuator.MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = true;

        public static final double GOBILDA312_CPR               = (((1.0 + (46.0/17.0))) * (1.0 + (46.0/11.0))) * 28.0;
        public static final double DEG_PER_COUNT                = 360.0 / GOBILDA312_CPR;
        public static final double POS_OFFSET                   = 39.0;
        public static final double POWER_LIMIT                  = 0.5;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 270.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {
            MIN_POS, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final boolean SQUID_ENABLED               = false;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.1, 0.001, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.158;
        // Since we don't have lower limit switch, must enable Stall Protection to do zero calibration by stalling.
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor motor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Arm()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            Params.posPidCoeffs, null, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED, Params.SQUID_ENABLED);
        motor.setPositionPidPowerComp(this::getGravityComp);
        motor.setStallProtection(
            Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
    }   //Arm

    /**
     * This method returns the created Arm motor.
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
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getGravityComp(double currPower)
    {
        return Params.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(motor.getPosition()));
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
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        motor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
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
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.displayPrintf(
            lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f",
            Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
        return lineNum;
    }   //updateStatus

}   //class Arm
