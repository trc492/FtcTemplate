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

import ftclib.motor.FtcMotorActuator;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcPidController;

/**
 * This class implements an Elevator Subsystem.
 */
public class Elevator
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Elevator";

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final FtcMotorActuator.MotorType MOTOR_TYPE= FtcMotorActuator.MotorType.DcMotor;

        public static final boolean MOTOR_INVERTED              = true;
        public static final double INCHES_PER_COUNT             = 18.25/4941.0;
        public static final double POS_OFFSET                   = 10.875;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 30.25;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 20.0, 25.0, 30.0};
        public static final double POS_PRESET_TOLERANCE         = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.1;
        public static final double GRAVITY_COMP_POWER           = 0.0;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    private final TrcMotor elevatorMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
    {
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        elevatorMotor = new FtcMotorActuator(motorParams).getMotor();
        elevatorMotor.setSoftwarePidEnabled(Params.SOFTWARE_PID_ENABLED);
        elevatorMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE);
        // elevatorMotor.setPositionPidPowerComp(this::getGravityComp);
        elevatorMotor.setStallProtection(
            Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
    }

    public TrcMotor getElevatorMotor()
    {
        return elevatorMotor;
    }

    // private double getGravityComp(double currPower)
    // {
    //     return RobotParams.Elevator.GRAVITY_COMP_POWER;
    // }

}   //class Elevator
