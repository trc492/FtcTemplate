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

package teamcode;

import androidx.annotation.NonNull;

import java.util.stream.Stream;

import ftclib.drivebase.FtcRobotBase;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.robotcore.FtcOpMode;
import ftclib.sensor.FtcRobotBattery;
import ftclib.vision.FtcLimelightVision;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.DriveBase;
import teamcode.vision.Vision;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.subsystem.TrcSubsystem;
import trclib.vision.TrcVisionRelocalize;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    public enum RelocalizationMode
    {
        Disabled,
        OneShot,
        Continuous
    }   //enum RelocalizationMode

    private final String moduleName = getClass().getSimpleName();
    // Global objects.
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    // Robot Drive.
    public DriveBase robotDriveBase;
    public FtcRobotBase.RobotInfo robotInfo;
    public FtcRobotBase robotBase;
    // Sensors and indicators.
    public FtcRobotBattery battery;
    public LEDIndicator ledIndicator;
    // Vision subsystems.
    public Vision vision;
    public TrcVisionRelocalize trcVisionRelocalize = null;
    private RelocalizationMode relocalizationMode = RelocalizationMode.Disabled;
    // Subsystems.
    // Autotasks.
    public TrcPose2D relocalizedRobotPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects.
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        speak("Init starting");
        // Create and initialize Robot Base.
        robotDriveBase = new DriveBase();
        robotInfo = robotDriveBase.getRobotInfo();
        robotBase = robotDriveBase.getRobotBase();
        // Create and initialize sensors and indicators.
        battery = RobotParams.Preferences.useBatteryMonitor? new FtcRobotBattery(): null;
        ledIndicator = RobotParams.Preferences.useLED && robotInfo.indicatorNames != null?
            new LEDIndicator(robotInfo.indicatorNames): null;
        // Create and initialize vision subsystems.
        if (RobotParams.Preferences.useVision && robotInfo.camInfos != null &&
            (RobotParams.Preferences.useLimelightVision ||
             RobotParams.Preferences.useWebcamAprilTagVision ||
             RobotParams.Preferences.useColorBlobVision))
        {
            vision = new Vision(this);
            if (RobotParams.Preferences.visionRelocalizeEnabled && robotBase != null)
            {
                trcVisionRelocalize = new TrcVisionRelocalize(100);
            }
        }
        // If robotType is VisionOnly, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != DriveBase.RobotType.VisionOnly)
        {
            if (RobotParams.Preferences.useSubsystems)
            {
                // Create subsystems.

                // Create autotasks.

                // Zero calibrate all subsystems only in Auto or if TeleOp is run standalone without prior Auto.
                // There is no reason to zero calibrate again if Auto was run right before TeleOp.
                if (runMode == TrcRobot.RunMode.AUTO_MODE || FtcAuto.autoChoices.alliance == null)
                {
                    zeroCalibrate(null, null);
                }
            }
        }
        speak("Init complete");
        Dashboard.DashboardParams.updateDashboardEnabled = RobotParams.Preferences.updateDashboard;
        if (Dashboard.DashboardParams.updateDashboardEnabled)
        {
            dashboard.enableDashboardUpdate(1, true);
        }
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return robotInfo != null? robotInfo.robotName: RobotParams.Preferences.robotType.toString();
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotBase != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotBase.gyro != null)
            {
                robotBase.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotBase.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotBase.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE && endOfAutoRobotPose != null)
            {
                // We had a previous autonomous run that saved the robot position at the end, use it.
                robotBase.driveBase.setFieldPosition(endOfAutoRobotPose);
                globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }

        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        // Stop everything.
        cancelAll();
        if (robotBase != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotBase.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            // Disable odometry.
            robotBase.driveBase.setOdometryEnabled(false);
            // Disable gyro task.
            if (robotBase.gyro != null)
            {
                robotBase.gyro.setEnabled(false);
            }
        }
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.isLimelightVisionEnabled())
            {
                globalTracer.traceInfo(moduleName, "Disabling LimelightVision.");
                vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.AprilTag, false);
            }

            if (vision.isWebcamAprilTagVisionEnabled())
            {
                globalTracer.traceInfo(moduleName, "Disabling Webcam AprilTagVision.");
                vision.setWebcamAprilTagVisionEnabled(false);
            }

            if (vision.backCamColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling ColorBlobVision.");
                vision.setColorBlobVisionEnabled(Vision.ColorBlobType.Any, false);
            }

            vision.close();
       }

        if (ledIndicator != null)
        {
            globalTracer.traceInfo(moduleName, "Turning all LED indicators OFF.");
            ledIndicator.reset();
        }
        //
        // Print all performance counters if there are any.
        //
        if (robotBase != null && robotBase.gyro != null)
        {
            robotBase.gyro.printElapsedTime(globalTracer);
            robotBase.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put code here that requires to be
     * run regardless of RobotMode (autonomous or teleop).
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (relocalizationMode != RelocalizationMode.Disabled)
        {
            if (relocalizeRobot() && relocalizationMode == RelocalizationMode.OneShot)
            {
                relocalizationMode = RelocalizationMode.Disabled;
            }
        }
    }   //periodic

    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");
        // Cancel auto tasks.
        TrcAutoTask.cancelAllTasks();
        // Cancel subsystems.
        if (robotBase != null) robotBase.cancel();
        TrcSubsystem.cancelAll();
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        globalTracer.traceInfo(moduleName, "Zero calibrate all subsystems.");
        TrcSubsystem.zeroCalibrateAll(owner, completionEvent);
    }   //zeroCalibrate

    /**
     * This method retracts all appendages for robot high speed traveling.
     */
    public void turtle()
    {
        globalTracer.traceInfo(moduleName, "Turtle mode.");
        TrcSubsystem.resetStateAll();
    }   //turtle

    /**
     * This method relocalizes the robot using vision.
     *
     * @return true if vision sees AprilTag and relocalize successfully, false otherwise.
     */
    public boolean relocalizeRobot()
    {
        boolean seenAprilTag = false;

        if (vision != null && RobotParams.Preferences.visionRelocalizeEnabled && trcVisionRelocalize != null)
        {
            TrcPose2D robotPose = robotBase.driveBase.getFieldPosition();
            long currTimestampMilli = System.currentTimeMillis();
            trcVisionRelocalize.addTimedPose(currTimestampMilli, robotPose);
            // Assume we are using Limelight to detect AprilTag.
            double limelightYaw = -robotPose.angle;
            limelightYaw = (limelightYaw + 180.0) % 360.0;
            if (limelightYaw < 0) limelightYaw += 360.0;
            limelightYaw -= 180.0;
            vision.limelightVision.updateRobotHeading(limelightYaw);
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagObj =
                vision.getLimelightDetectedObject(FtcLimelightVision.ResultType.Fiducial, null, null, -1);

            if (aprilTagObj != null)
            {
                seenAprilTag = true;
                TrcPose2D robotVel = robotBase.driveBase.getRobotVelocity();
                TrcPose2D relocalizedPose =
                    Math.hypot(robotVel.x, robotVel.y) > 0.01 || Math.abs(robotVel.angle) > 1.0?
                        trcVisionRelocalize.getRelocalizedPose(
                            aprilTagObj.detectedObj.timestamp, aprilTagObj.detectedObj.robotPose, robotPose):
                        aprilTagObj.detectedObj.robotPose;

                robotBase.driveBase.setFieldPosition(relocalizedPose);
                globalTracer.traceDebug(
                    moduleName,
                    "VisionRelocalize: TimeMilli=%d, Relocalize %s->%s, VisionPose[%d](time=%d, pose=%s)",
                    currTimestampMilli, robotPose, relocalizedPose, (int)aprilTagObj.detectedObj.objId,
                    (long)aprilTagObj.detectedObj.timestamp, aprilTagObj.detectedObj.robotPose);
            }

            if (ledIndicator != null)
            {
                ledIndicator.setStatusPattern(LEDIndicator.RED_APRILTAG, false);
                ledIndicator.setStatusPattern(LEDIndicator.BLUE_APRILTAG, false);
                if (seenAprilTag)
                {
                    ledIndicator.setStatusPattern(
                        (int) aprilTagObj.detectedObj.objId == RobotParams.Game.RED_APRILTAG_ID?
                            LEDIndicator.RED_APRILTAG:
                        (int) aprilTagObj.detectedObj.objId == RobotParams.Game.BLUE_APRILTAG_ID?
                            LEDIndicator.BLUE_APRILTAG: LEDIndicator.NOT_FOUND,
                        true);
                }
            }
        }

        return seenAprilTag;
    }   //relocalizeRobot

    /**
     * This method sets the relocalization mode.
     *
     * @param relocalizationMode specifies the relocalization mode.
     */
    public void setRelocalizationMode(RelocalizationMode relocalizationMode)
    {
        globalTracer.traceInfo(moduleName, "setRelocalizationMode to " + relocalizationMode);
        this.relocalizationMode = relocalizationMode;
    }   //setRelocalizationMode

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in the specified unit.
     * @param y specifies y position in the red alliance in the specified unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if x and y are in tile unit, false if in inches.
     * @param relativePose specifies if the pose is a relative pose.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit, boolean relativePose)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.Blue)
        {
            if (relativePose)
            {
                if (!RobotParams.Field.mirroredField)
                {
                    newPose.x = -newPose.x;
                }
            }
            // Translate blue alliance pose to red alliance pose.
            else if (RobotParams.Field.mirroredField)
            {
                // Field is mirrored on X axis.
                // Same X, Flip Y. Heading left becomes right and right becomes left.
                double angleDelta = (newPose.angle - 90.0)*2.0;
                newPose.angle -= angleDelta;
                newPose.y = -newPose.y;
            }
            else
            {
                // Field is symmetrical.
                // Flip X, Flip Y. Heading flips 180-degree.
                newPose.x = -newPose.x;
                newPose.y = -newPose.y;
                newPose.angle = (newPose.angle + 180.0) % 360.0;
            }
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.Field.fullTileInches;
            newPose.y *= RobotParams.Field.fullTileInches;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in the specified unit.
     * @param y specifies y position in the red alliance in the specified unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if x and y are in tile unit, false if in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, isTileUnit, false);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in tile unit.
     * @param y specifies y position in the red alliance in tile unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, false, false);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in the specified unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false in inches.
     * @param relativePose specifies if the pose is a relative pose.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit, boolean relativePose)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit, relativePose);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in the specified unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit, false);
    }   //adjustPoseByAlliance

    public TrcPose2D[] adjustPathByAlliance(
        FtcAuto.Alliance alliance, boolean isTileUnit, boolean relativePose, TrcPose2D... poses)
    {
        return Stream.of(poses)
                     .map(pose -> adjustPoseByAlliance(pose, alliance, isTileUnit, relativePose))
                     .toArray(TrcPose2D[]::new);
    }   //adjustPathByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, false, false);
    }   //adjustPoseByAlliance

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
