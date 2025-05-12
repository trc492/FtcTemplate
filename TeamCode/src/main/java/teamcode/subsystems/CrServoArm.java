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
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a CrServoArm Subsystem. This implementation consists of two Axon servos running in Continuous
 * Rotation mode with an analog absolute encoder. It does not require zero calibration. Therefore, limit switches are
 * optional. If using limit switches, they are for movement range protection. If not using limit switches, software
 * limit must be set. It supports gravity compensation by computing the power required to hold the arm at its current
 * angle.
 * There are many possible implementations by setting different parameters.
 * Please refer to the TrcLib documentation (<a href="https://trc492.github.io">...</a>) for details.
 */
public class CrServoArm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "CrServoArm";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE =
            FtcMotorActuator.MotorType.CRServo;
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final FtcMotorActuator.MotorType FOLLOWER_MOTOR_TYPE =
            FtcMotorActuator.MotorType.CRServo;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;

        public static final double POWER_LIMIT                  = 0.25;

        public static final String ABSENC_NAME                  = SUBSYSTEM_NAME + ".enc";
        public static final boolean ABSENC_INVERTED             = true;
        public static final double ABSENC_ZERO_OFFSET           = 0.949697;

        public static final double POS_OFFSET                   = 27.0;
        public static final double POS_DEG_SCALE                = 360.0;
        public static final double MIN_POS                      = 27.3;
        public static final double MAX_POS                      = 300.0;

        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {
            30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.0162, 0.0, 0.0, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.1675;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor motor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CrServoArm()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setFollowerMotor(Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED)
            .setExternalEncoder(Params.ABSENC_NAME, Params.ABSENC_INVERTED)
            .setPositionScaleAndOffset(Params.POS_DEG_SCALE, Params.POS_OFFSET, Params.ABSENC_ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        motor.setPositionPidPowerComp(this::getGravityComp);
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //CrServoArm

    /**
     * This method returns the created CrServoArm motor.
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
        return Params.GRAVITY_COMP_MAX_POWER*Math.sin(Math.toRadians(motor.getPosition()));
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
        // No zero calibration needed.
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

}   //class CrServoArm
