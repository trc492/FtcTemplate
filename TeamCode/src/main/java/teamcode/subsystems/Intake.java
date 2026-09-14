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
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.subsystem.FtcRollerIntake;
import teamcode.FtcAuto;
import teamcode.Robot;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcSubsystem;
import trclib.subsystem.TrcRollerIntake.TriggerAction;

/**
 * This class implements an Intake Subsystem. This implementation consists of one or two motors and optionally a
 * front and/or back digital sensor(s) that can detect object entering/exiting the intake.
 */
public class Intake extends TrcSubsystem<Intake.Action>
{
    public static final String SUBSYSTEM_NAME = "Intake";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        private static final boolean HAS_TWO_MOTORS             = false;
        private static final boolean HAS_FRONT_SENSOR           = false;
        private static final boolean HAS_BACK_SENSOR            = true;

        public static final MotorType MOTOR_TYPE                = MotorType.DcMotor;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;
        public static final boolean PRIMARY_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean PRIMARY_MOTOR_BRAKE_ENABLED = true;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_MOTOR_INVERTED     = !PRIMARY_MOTOR_INVERTED;
        public static final boolean FOLLOWER_MOTOR_VOLTCOMP_ENABLED = true;
        public static final boolean FOLLOWER_MOTOR_BRAKE_ENABLED= true;

        public static final String FRONT_SENSOR_NAME            = SUBSYSTEM_NAME + ".frontSensor";
        public static final boolean FRONT_SENSOR_INVERTED       = false;

        public static final String BACK_SENSOR_NAME             = SUBSYSTEM_NAME + ".backSensor";
        public static final boolean BACK_SENSOR_INVERTED        = false;

        public static final double INTAKE_POWER                 = 1.0;  // Intake forward
        public static final double EJECT_POWER                  = 1.0;  // Eject forward
        public static final double RETAIN_POWER                 = 0.0;
        public static final double INTAKE_FINISH_DELAY          = 0.0;
        public static final double EJECT_FINISH_DELAY           = 0.5;
    }   //class Params

    public enum Action
    {
        ToggleAutoPickup,
        ToggleManualIntake,
        ToggleSensorIntake
    }   //enum Action

    private final Robot robot;
    private final FtcDashboard dashboard;
    private final TrcRollerIntake intake;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access other subsystems if necessary.
     */
    public Intake(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.robot = robot;
        dashboard = FtcDashboard.getInstance();
        FtcRollerIntake.Params intakeParams = new FtcRollerIntake.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED,
                Params.PRIMARY_MOTOR_VOLTCOMP_ENABLED, Params.PRIMARY_MOTOR_BRAKE_ENABLED)
            .setPowerLevels(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setFinishDelays(Params.INTAKE_FINISH_DELAY, Params.EJECT_FINISH_DELAY);

        if (Params.HAS_TWO_MOTORS)
        {
            intakeParams.addFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED,
                Params.FOLLOWER_MOTOR_VOLTCOMP_ENABLED, Params.FOLLOWER_MOTOR_BRAKE_ENABLED);
        }

        if (Params.HAS_FRONT_SENSOR)
        {
            intakeParams.setFrontDigitalInputTrigger(
                Params.FRONT_SENSOR_NAME, Params.FRONT_SENSOR_INVERTED, TriggerAction.NoAction, null, null, null);
        }

        if (Params.HAS_BACK_SENSOR)
        {
            intakeParams.setBackDigitalInputTrigger(
                Params.BACK_SENSOR_NAME, Params.BACK_SENSOR_INVERTED, TriggerAction.FinishOnTrigger, null, null, null);
        }
        intake = new FtcRollerIntake(SUBSYSTEM_NAME, intakeParams).getIntake();
    }   //Intake

    /**
     * This method returns the created TrcRollerIntake.
     *
     * @return created Roller Intake.
     */
    public TrcRollerIntake getIntake()
    {
        return intake;
    }   //getIntake

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        intake.cancel();
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
        // Intake does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Intake does not support resetState.
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
    }   //subsystemControl

    /**
     * This method is called to perform the subsystem action.
     *
     * @param action specifies the subsystem action to perform.
     * @param context specifies the context object for the action.
     */
    @Override
    public void subsystemAction(Action action, Object context)
    {
        switch (action)
        {
            case ToggleAutoPickup:
                if (robot.autoPickupTask.isActive())
                {
                    robot.autoPickupTask.cancel();
                    intake.tracer.traceInfo(instanceName, ">>>>> Cancel Auto Pickup.");
                }
                else
                {
                    boolean useVision = context != null && (Boolean) context;

                    robot.autoPickupTask.autoPickup(
                        instanceName, null, FtcAuto.autoChoices.alliance, useVision);
                    intake.tracer.traceInfo(
                        instanceName, ">>>>> Auto Pickup (useVision=" + useVision + ").");
                }
                break;

            case ToggleSensorIntake:
                if (intake.isAutoActive())
                {
                    intake.cancel();
                    intake.tracer.traceInfo(instanceName, ">>>>> Cancel Sensor Intake.");
                }
                else
                {
                    intake.autoIntake(instanceName);
                    intake.tracer.traceInfo(instanceName, ">>>>> Sensor Intake.");
                }
                break;

            case ToggleManualIntake:
                if (intake.getPower() == 0.0)
                {
                    intake.setPower(Params.INTAKE_POWER);
                    intake.tracer.traceInfo(instanceName, ">>>>> Manual Intake.");
                }
                else
                {
                    intake.cancel();
                    intake.tracer.traceInfo(instanceName, ">>>>> Cancel Manual Intake.");
                }
                break;

            default:
                break;
        }
    }   //subsystemAction

    /**
     * This method is called to perform the subsystem tune action.
     *
     * @param action specifies the subsystem tune action to perform.
     * @param tuneSubsystemName specifies the subsystem object to tune.
     */
    public void tuneSubsystem(TuneAction action, String tuneSubsystemName)
    {
        // Intake subsystem doesn't need tuning.
    }   //tuneSubsystem

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
                lineNum++, "%s: power=%.3f, current=%.3f, hasObject=%s, sensorState=%s/%s, autoActive=%s",
                SUBSYSTEM_NAME, intake.getPower(), intake.getCurrent(), intake.hasObject(),
                intake.getFrontSensorState(), intake.getBackSensorState(), intake.isAutoActive());
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
        // Intake subsystem doesn't need tuning.
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
        // Intake subsystem doesn't need tuning.
    }   //updateParamsFromDashboard

}   //class Intake
