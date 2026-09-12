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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.driverio.FtcDashboard;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoClaw;
import teamcode.Dashboard;
import teamcode.Robot;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.subsystem.TrcServoClaw;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Claw Subsystem. This implementation consists of two servos and a sensor to detect the
 * presence of an object and can auto grab it. The sensor can be either a digital sensor such as touch sensor or beam
 * break sensor) or an analog sensor such as a distance sensor.
 */
public class ServoClaw extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "ServoClaw";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        private static final boolean USE_ANALOG_SENSOR          = true;
        private static final boolean USE_DIGITAL_SENSOR         = false;

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".leftClaw";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".rightClaw";
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final String ANALOG_SENSOR_NAME           = SUBSYSTEM_NAME + ".sensor";
        public static final double LOWER_TRIGGER_THRESHOLD      = 2.0;
        public static final double UPPER_TRIGGER_THRESHOLD      = 3.0;
        public static final double TRIGGER_SETTLING_TIME        = 0.1;

        public static final String DIGITAL_SENSOR_NAME          = SUBSYSTEM_NAME + ".sensor";
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;
    }   //class Params

    private final Robot robot;
    private final FtcDashboard dashboard;
    private final Rev2mDistanceSensor analogSensor;
    private final TrcServoClaw claw;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access other subsystems if necessary.
     */
    public ServoClaw(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.robot = robot;
        dashboard = FtcDashboard.getInstance();
        if (Params.USE_ANALOG_SENSOR)
        {
            analogSensor =
                FtcOpMode.getInstance().hardwareMap.get(Rev2mDistanceSensor.class, Params.ANALOG_SENSOR_NAME);
        }
        else
        {
            analogSensor = null;
        }

        FtcServoClaw.Params clawParams = new FtcServoClaw.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME);

        if (analogSensor != null)
        {
            clawParams.setAnalogSourceTrigger(
                Params.ANALOG_SENSOR_NAME, this::getSensorData,
                new TrcTriggerThresholdRange.TriggerParams(
                    Params.LOWER_TRIGGER_THRESHOLD, Params.UPPER_TRIGGER_THRESHOLD, Params.TRIGGER_SETTLING_TIME));
        }
        else if (Params.USE_DIGITAL_SENSOR)
        {
            clawParams.setDigitalInputTrigger(Params.DIGITAL_SENSOR_NAME, Params.DIGITAL_TRIGGER_INVERTED);
        }

        claw = new FtcServoClaw(SUBSYSTEM_NAME, clawParams).getClaw();
        claw.open();
    }   //ServoClaw

    /**
     * This method returns the created Servo Claw object.
     *
     * @return created claw.
     */
    public TrcServoClaw getClaw()
    {
        return claw;
    }   //getClaw

     /**
      * This method returns the current sensor value if it has one.
      *
      * @return sensor value if there is a sensor, 0.0 if there is none.
      */
     private double getSensorData()
     {
         if (analogSensor != null)
         {
             return analogSensor.getDistance(DistanceUnit.INCH);
         }
         else
         {
             return 0.0;
         }
     }   //getSensorData

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        claw.cancel();
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
        // Don't move claw during turtle.
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
     * This method is called when a gamepad button is pressed to perform the subsystem action.
     *
     * @param pressed specifies true if the gamepad button is pressed, false otherwise.
     * @param altFunc specifies true if the gamepad AltFunc button is pressed, false otherwise.
     */
    @Override
    public void subsystemAction(boolean pressed, boolean altFunc)
    {
        if (pressed)
        {
            if (altFunc)
            {
                if (claw.isClosed())
                {
                    claw.open();
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Opening claws");
                }
                else
                {
                    claw.close();
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Closing claws");
                }
            }
            else
            {
                if (claw.isAutoActive() || claw.hasObject())
                {
                    claw.cancel();
                    claw.open();
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Canceling AutoGrab.");
                }
                else
                {
                    claw.autoGrab(null, 0.0, null, 0.0);
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Enabling AutoGrab.");
                }
            }
        }
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
                lineNum++, "%s: pos=%s, closed=%s, hasObject=%s, sensorState=%s, sensorValue=%.3f, autoActive=%s",
                SUBSYSTEM_NAME, claw.getPosition(), claw.isClosed(), claw.hasObject(), claw.getTriggerState(),
                claw.getSensorValue(), claw.isAutoActive());
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            Dashboard.TuneSubsystem.target = Params.OPEN_POS;
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            claw.setPosition(null, 0.0, Dashboard.TuneSubsystem.target, null, 0.0);
            claw.tracer.traceInfo(
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            double target = Params.OPEN_POS;
            claw.setPosition(null, 0.0, target, null, 0.0);
            claw.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, target);
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            double target = Params.CLOSE_POS;
            claw.setPosition(null, 0.0, target, null, 0.0);
            claw.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class ServoClaw
