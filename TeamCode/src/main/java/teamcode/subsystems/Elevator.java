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
 * This class implements an Elevator Subsystem. This implementation consists of two motors with built-in encoders and
 * a lower limit switch for zero calibrating the relative encoder. It supports gravity compensation. In the case of
 * an elevator, gravity compensation power is the constant power required to hold the elevator at any position.
 * There are many possible implementations by setting different parameters.
 * Please refer to the TrcLib documentation (<a href="https://trc492.github.io">...</a>) for details.
 */
public class Elevator extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Elevator";
        public static final boolean NEED_ZERO_CAL               = true;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE =
            FtcMotorActuator.MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final FtcMotorActuator.MotorType FOLLOWER_MOTOR_TYPE =
            FtcMotorActuator.MotorType.DcMotor;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;

        public static final String LOWER_LIMITSW_NAME           = SUBSYSTEM_NAME + ".lowerLimitSw";
        public static final boolean LOWER_LIMITSW_INVERTED      = false;

        public static final double INCHES_PER_COUNT             = 18.25/4941.0;
        public static final double POS_OFFSET                   = 10.875;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 30.25;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 20.0, 25.0, 30.0};
        public static final double POS_PRESET_TOLERANCE         = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.1;
        public static final double GRAVITY_COMP_POWER           = 0.0;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor motor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setFollowerMotor(Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMITSW_NAME, Params.LOWER_LIMITSW_INVERTED)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        motor.setPositionPidPowerComp(this::getGravityComp);
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
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
     private double getGravityComp(double currPower)
     {
         return Params.GRAVITY_COMP_POWER;
     }  //getGravityComp

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
            lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f, lowerLimit=%s",
            Params.SUBSYSTEM_NAME, motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget(),
            motor.isLowerLimitSwitchActive());
        return lineNum;
    }   //updateStatus

}   //class Elevator
