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

package teamcode.seafairdemo;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcDcMotor;
import ftclib.subsystem.FtcMotorActuator;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcMecanumDriveBase;
import trclib.drivebase.TrcSimpleDriveBase;
import trclib.motor.TrcMotor;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    // DriveBase Subsystem
    private static final int INDEX_LEFT_FRONT = 0;
    private static final int INDEX_RIGHT_FRONT = 1;
    private static final int INDEX_LEFT_BACK = 2;
    private static final int INDEX_RIGHT_BACK = 3;
    private static final String[] driveMotorNames = {"lfDriveWheel", "rfDriveWheel", "lbDriveWheel", "rbDriveWheel"};
    private static final boolean[] driveMotorInverted = {true, false, true, false};
    public static final double DRIVE_POWER_SCALE = 0.5;
    public static final double TURN_POWER_SCALE = 0.5;
    // Elevator Subsystem
    private static final String ELEVATOR_NAME = "elevator";
    private static final boolean ELEVATOR_MOTOR_INVERTED = false;
    private static final boolean ELEVATOR_HAS_LOWER_LIMIT_SWITCH = true;
    private static final boolean ELEVATOR_LOWER_LIMIT_SWITCH_INVERTED = true;
    private static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH = true;
    private static final boolean ELEVATOR_UPPER_LIMIT_SWITCH_INVERTED = true;
    private static final double ELEVATOR_INCHES_PER_COUNT = 5.625/8498.0;
    private static final double ELEVATOR_OFFSET = 16.25;
    private static final double ELEVATOR_TOLERANCE = 0.2;
    private static final double ELEVATOR_CAL_POWER = -0.3;
    private static final double ELEVATOR_MIN_HEIGHT = ELEVATOR_OFFSET - 0.1;
    private static final double ELEVATOR_MAX_HEIGHT = 23.0;

    public final FtcDashboard dashboard;
    public final TrcDriveBase driveBase;
    public final TrcMotor elevator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param roverRuckusRobot specifies true for the Rover Ruckus drive base, false for differential drive base.
     */
    public Robot(boolean roverRuckusRobot)
    {
        dashboard = FtcDashboard.getInstance();

        FtcDcMotor[] driveMotors = new FtcDcMotor[driveMotorNames.length];
        for (int i = 0; i < driveMotorNames.length; i++)
        {
            driveMotors[i] = new FtcDcMotor(driveMotorNames[i]);
            driveMotors[i].setMotorInverted(driveMotorInverted[i]);
            driveMotors[i].setBrakeModeEnabled(true);
            driveMotors[i].setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        }

        if (roverRuckusRobot)
        {
            // Rover Ruckus robot has a mecanum drive base and an elevator with lower and upper limit switches.
            driveBase = new TrcMecanumDriveBase(
                driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
                driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK]);
            driveBase.setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT, false);
            FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
                .setMotorInverted(ELEVATOR_MOTOR_INVERTED)
                .setLowerLimitSwitch(ELEVATOR_HAS_LOWER_LIMIT_SWITCH, ELEVATOR_LOWER_LIMIT_SWITCH_INVERTED)
                .setUpperLimitSwitch(ELEVATOR_HAS_UPPER_LIMIT_SWITCH, ELEVATOR_UPPER_LIMIT_SWITCH_INVERTED)
                .setVoltageCompensationEnabled(true)
                .setPositionScaleAndOffset(ELEVATOR_INCHES_PER_COUNT, ELEVATOR_OFFSET);
            elevator = new FtcMotorActuator(ELEVATOR_NAME, elevatorParams).getActuator();
            elevator.setBrakeModeEnabled(true);
            elevator.setPositionPidTolerance(ELEVATOR_TOLERANCE);
        }
        else
        {
            // Differential drive base is a West Coast drive base with 6 wheels and four drive motors.
            driveBase = new TrcSimpleDriveBase(
                driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
                driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK]);
            elevator = null;
        }
    }   //Robot

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        if (elevator != null)
        {
            elevator.zeroCalibrate(ELEVATOR_CAL_POWER);
        }
    }   //zeroCalibrate

    /**
     * This method sets the elevator raw power when manual override is ON.
     *
     * @param power specifies elevator power applied.
     */
    public void setElevatorPower(double power)
    {
        if (elevator != null)
        {
            elevator.setPower(power);
        }
    }   //setElevatorPower

    /**
     * This method sets the elevator pid power using PID control and will slow down when approaching limits.
     *
     * @param power specifies elevator power applied.
     */
    public void setElevatorPidPower(double power)
    {
        if (elevator != null)
        {
            elevator.setPidPower(power, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT, true);
        }
    }   //setElevatorPidPower

}   //class Robot
