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

import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import trclib.driverio.TrcGameController;

/**
 * This class contains the Common TeleOp Mode code and is extended by different TeleOp OpModes.
 */
public abstract class CommonTeleOp extends FtcOpMode
{
    private Robot robot;
    private FtcGamepad driverGamepad;
    private boolean manualOverride = false;

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     *
     * @param roverRuckusRobot specifies true for the Rover Ruckus drive base, false for differential drive base.
     */
    public void teleOpInit(boolean roverRuckusRobot)
    {
        robot = new Robot(roverRuckusRobot);
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        driverGamepad.setYInverted(true);
    }   //teleOpInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            int lineNum = 1;
            //
            // DriveBase subsystem.
            //
            double[] inputs = driverGamepad.getDriveInputs(
                FtcGamepad.DriveMode.ARCADE_MODE, true, Robot.DRIVE_POWER_SCALE, Robot.TURN_POWER_SCALE);
            if (robot.driveBase.supportsHolonomicDrive())
            {
                robot.driveBase.holonomicDrive(inputs[0], inputs[1], inputs[2]);
            }
            else
            {
                robot.driveBase.arcadeDrive(inputs[1], inputs[2]);
            }
            robot.dashboard.displayPrintf(
                lineNum++, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f)", inputs[0], inputs[1], inputs[2]);
            //
            // Other subsystems.
            //
            if (robot.elevator != null)
            {
                double elevatorPower = driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true);
                if (manualOverride)
                {
                    robot.setElevatorPower(elevatorPower);
                }
                else
                {
                    robot.setElevatorPidPower(elevatorPower);
                }
                robot.dashboard.displayPrintf(
                    lineNum++, "Elevator: power=%.1f, pos=%.1f/%.1f, limitSw=%s/%s",
                    robot.elevator.getPower(), robot.elevator.getPosition(), robot.elevator.getPidTarget(),
                    robot.elevator.isLowerLimitSwitchActive(), robot.elevator.isUpperLimitSwitchActive());
            }
        }
    }   //periodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
            case FtcGamepad.GAMEPAD_B:
            case FtcGamepad.GAMEPAD_X:
            case FtcGamepad.GAMEPAD_Y:
            case FtcGamepad.GAMEPAD_LBUMPER:
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
//                manualOverride = pressed;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
            case FtcGamepad.GAMEPAD_DPAD_DOWN:
            case FtcGamepad.GAMEPAD_DPAD_LEFT:
            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
            case FtcGamepad.GAMEPAD_START:
                break;

            case FtcGamepad.GAMEPAD_BACK:
                robot.zeroCalibrate();
                break;
        }
    }   //driverButtonEvent

}   //class CommonTeleOp
