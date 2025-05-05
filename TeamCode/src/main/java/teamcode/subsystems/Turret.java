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
 * This class implements a Turret Subsystem. This implementation consists of a motor with built-in encoder. It has
 * a lower limit switch for zero calibrating the built-in relative encoder. Since Turret is circular in nature, it
 * is recommended to implement a hard stop to prevent the Turret from overrunning the upper limit causing the wiring
 * harness to be twisted. Even though we do implement soft limits on the Turret, hard stop would prevent folks from
 * spinning the turret round and round twisting the wiring harness when the robot is off.
 * There are many possible implementations by setting different parameters.
 * Please refer to the TrcLib documentation (<a href="https://trc492.github.io">...</a>) for details.
 */
public class Turret extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Turret";
        public static final boolean NEED_ZERO_CAL               = true;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final FtcMotorActuator.MotorType MOTOR_TYPE= FtcMotorActuator.MotorType.DcMotor;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String LOWER_LIMITSW_NAME           = SUBSYSTEM_NAME + ".lowerLimitSw";
        public static final boolean LOWER_LIMITSW_INVERTED      = false;

        public static final double ENCODER_PPR                  = 288.0;
        public static final double GEAR_RATIO                   = 100.0/60.0;
        public static final double DEG_PER_COUNT                = 360.0/(ENCODER_PPR*GEAR_RATIO);
        public static final double POS_OFFSET                   = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.3;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 325.0;
        public static final double BACK                         = 0.0;
        public static final double RIGHT                        = 90.0;
        public static final double FRONT                        = 180.0;
        public static final double LEFT                         = 270.0;
        public static final double TURTLE_POS                   = FRONT;
        public static final double TURTLE_DELAY                 = 0.0;

        // Preset positions.
        public static final double[] posPresets                 = new double[] {BACK, RIGHT, FRONT, LEFT};
        public static final double POS_PRESET_TOLERANCE         = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.04, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor motor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Turret()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMITSW_NAME, Params.LOWER_LIMITSW_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FtcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        // Since we don't have upper limit switch, setting soft limits will protect turret from overrunning the upper
        // limit in manual mode.
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //Turret

    /**
     * This method returns the created Turret motor.
     *
     * @return created turret motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

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

}   //class Turret
