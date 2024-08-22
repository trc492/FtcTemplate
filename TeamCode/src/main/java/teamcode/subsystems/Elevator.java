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
import teamcode.RobotParams;
import trclib.motor.TrcMotor;

/**
 * This class implements an Elevator Subsystem.
 */
public class Elevator
{
    private final TrcMotor elevatorMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
    {
        FtcMotorActuator.Params motorParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(RobotParams.Elevator.MOTOR_NAME, RobotParams.Elevator.MOTOR_TYPE,
                             RobotParams.Elevator.MOTOR_INVERTED)
            .setPositionScaleAndOffset(RobotParams.Elevator.INCHES_PER_COUNT, RobotParams.Elevator.POS_OFFSET)
            .setPositionPresets(RobotParams.Elevator.POS_PRESET_TOLERANCE, RobotParams.Elevator.posPresets);
        elevatorMotor = new FtcMotorActuator(motorParams).getMotor();
        elevatorMotor.setSoftwarePidEnabled(RobotParams.Elevator.SOFTWARE_PID_ENABLED);
        elevatorMotor.setPositionPidParameters(
            RobotParams.Elevator.posPidCoeffs, RobotParams.Elevator.POS_PID_TOLERANCE);
        // elevatorMotor.setPositionPidPowerComp(this::getGravityComp);
        elevatorMotor.setStallProtection(
            RobotParams.Elevator.STALL_MIN_POWER, RobotParams.Elevator.STALL_TOLERANCE,
            RobotParams.Elevator.STALL_TIMEOUT, RobotParams.Elevator.STALL_RESET_TIMEOUT);
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
