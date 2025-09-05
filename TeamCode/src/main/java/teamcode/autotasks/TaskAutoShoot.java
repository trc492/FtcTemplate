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

package teamcode.autotasks;

import androidx.annotation.NonNull;

import java.util.Arrays;

import ftclib.vision.FtcVisionAprilTag;
import teamcode.Robot;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.Shooter;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcShootParamTable;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoShoot extends TrcAutoTask<TaskAutoShoot.State>
{
    private static final String moduleName = TaskAutoShoot.class.getSimpleName();

    public enum State
    {
        START,
        FIND_APRILTAG,
        AIM_AND_SHOOT,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;
        int[] aprilTagIds;

        TaskParams(boolean useVision, int... aprilTagIds)
        {
            this.useVision = useVision;
            this.aprilTagIds = aprilTagIds;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "useVision=" + useVision + ", aprilTagIds=" + Arrays.toString(aprilTagIds);
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent event;

    private Double visionExpiredTime = null;
    private TrcPose2D aprilTagPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoShoot(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
    }   //TaskAutoShoot

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param useVision specifies true to use Vision, false otherwise.
     * @param aprilTagIds specifies multiple AprilTag IDs for vision to look for, can be null to look for any AprilTag.
     */
    public void autoShoot(String owner, TrcEvent completionEvent, boolean useVision, int... aprilTagIds)
    {
        TaskParams taskParams = new TaskParams(useVision, aprilTagIds);
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoShoot

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null ||
               robot.shooter.acquireExclusiveAccess(owner) &&
               (robot.intake == null || robot.intake.acquireExclusiveAccess(owner));
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\tshooter=" + ownershipMgr.getOwner(robot.shooter) +
                (robot.intake != null? ("\n\tintake=" + ownershipMgr.getOwner(robot.intake)): ""));
            robot.shooter.releaseExclusiveAccess(owner);
            if (robot.intake != null)
            {
                robot.intake.releaseExclusiveAccess(owner);
            }
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.shooter.cancel(owner);
        if (robot.intake != null)
        {
            robot.intake.cancel();
        }
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                aprilTagPose = null;
                if (!taskParams.useVision)
                {
                    // Not using AprilTag vision, skip vision and just score at current positon assuming operator
                    // has manually aimed at target.
                    tracer.traceInfo(moduleName, "***** Not using AprilTag Vision.");
                    sm.setState(State.AIM_AND_SHOOT);
                }
                else if (robot.vision != null && robot.vision.aprilTagVision != null)
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision.");
                    visionExpiredTime = null;
                    sm.setState(State.FIND_APRILTAG);
                }
                else
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision but Vision is not enabled.");
                    sm.setState(State.DONE);
                }
                break;

            case FIND_APRILTAG:
                // Use vision to determine the appropriate AprilTag location.
                TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> object =
                    robot.vision.aprilTagVision.getBestDetectedTargetInfo(null, null);
                if (object != null)
                {
                    int aprilTagId = object.detectedObj.aprilTagDetection.id;
                    tracer.traceInfo(
                        moduleName,
                        "***** Vision found AprilTag " + aprilTagId + ": aprilTagPose=" + object.objPose);
                    aprilTagPose = object.detectedObj.getObjectPose();
                    sm.setState(State.AIM_AND_SHOOT);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find AprilTag, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** No AprilTag found.");
                    if (robot.ledIndicator1 != null)
                    {
                        // Indicate we timed out and found nothing.
                        robot.ledIndicator1.setDetectedPattern(LEDIndicator.FOUND_NOTHING);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case AIM_AND_SHOOT:
                if (aprilTagPose != null)
                {
                    // Determine shooter speed, pan and tilt angle according to detected AprilTag pose.
                    // Use vision distance to look up shooter parameters.
                    double aprilTagDistance = TrcUtil.magnitude(aprilTagPose.x, aprilTagPose.y);
                    TrcShootParamTable.Params shootParams =
                        Shooter.Params.shootParamTable.get(aprilTagDistance, false);

                    robot.shooter.aimShooter(
                        owner, shootParams.shooter1Velocity, shootParams.shooter2Velocity, shootParams.tiltAngle,
                        aprilTagPose.angle, event, 0.0, robot.shooterSubsystem::shoot,
                        Shooter.Params.SHOOTER_OFF_DELAY);
                    tracer.traceInfo(
                        moduleName, "***** ShootParams: distance=" + aprilTagDistance + ", params=" + shootParams);
                    }
                else
                {
                    // We did not use vision, just shoot assuming operator manually aimed.
                    double shooterVel = robot.shooterSubsystem.shooter1Velocity.getValue();
                    // ShooterVel is in RPM, aimShooter wants RPS.
                    robot.shooter.aimShooter(
                        owner, shooterVel / 60.0, 0.0, null, null, event, 0.0,
                        robot.shooterSubsystem::shoot, Shooter.Params.SHOOTER_OFF_DELAY);
                    tracer.traceInfo(
                        moduleName, "***** ManualShoot: shooterVel=" + shooterVel + " RPM");
                }
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoShoot
