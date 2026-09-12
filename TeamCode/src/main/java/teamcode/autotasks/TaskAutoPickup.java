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

import ftclib.vision.FtcLimelightVision;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.indicators.LEDIndicator;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = TaskAutoPickup.class.getSimpleName();

    public enum State
    {
        START,
        FIND_OBJ,
        PICKUP_OBJ,
        DONE
    }   //enum State

    private static class TaskParams
    {
        FtcAuto.Alliance alliance;
        boolean useVision;

        TaskParams(FtcAuto.Alliance alliance, boolean useVision)
        {
            this.alliance = alliance;
            this.useVision = useVision;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "alliance=" + alliance + ", useVision=" + useVision;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent pickupEvent;
    private final TrcEvent driveEvent;

    private TaskParams taskParams = null;
    private Double visionExpiredTime = null;
    private TrcPose2D objPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickup(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.pickupEvent = new TrcEvent(moduleName + ".pickupEvent");
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
    }   //TaskAutoPickup

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param alliance specifies the alliance color for vision processing.
     * @param useVision specifies true to use Vision, false otherwise.
     */
    public void autoPickup(String owner, TrcEvent completionEvent, FtcAuto.Alliance alliance, boolean useVision)
    {
        taskParams = new TaskParams(alliance, useVision);
        tracer.traceInfo(
            moduleName,
            "autoPickup(owner=" + owner + ", event=" + completionEvent + ", taskParams=(" + taskParams + "))");
        startAutoTask(owner, State.START, completionEvent);
    }   //autoPickup

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
        boolean success = true;

        if (owner != null)
        {
            if (taskParams.useVision && robot.robotBase != null)
            {
                success = robot.robotBase.driveBase.acquireExclusiveAccess(owner);
            }

            success &= robot.intake.acquireExclusiveAccess(owner);
        }

        return success;
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
                "\n\tintake=" + ownershipMgr.getOwner(robot.intake) +
                (taskParams.useVision && robot.robotBase != null?
                    ("\n\tdriveBase=" + ownershipMgr.getOwner(robot.robotBase.driveBase)): ""));
            robot.intake.releaseExclusiveAccess(owner);
            if (taskParams.useVision && robot.robotBase != null)
            {
                robot.robotBase.driveBase.releaseExclusiveAccess(owner);
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
        robot.intake.cancel();
        if (taskParams.useVision && robot.robotBase != null)
        {
            robot.robotBase.cancel();
        }
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        switch (state)
        {
            case START:
                objPose = null;
                if (!taskParams.useVision)
                {
                    // Not using vision, just turn on intake and let the driver plow towards the object to pick it up
                    // manually.
                    tracer.traceInfo(moduleName, "***** Not using Vision, manual pickup.");
                    sm.setState(State.PICKUP_OBJ);
                }
                else if (robot.vision != null && robot.vision.limelightVision != null)
                {
                    tracer.traceInfo(moduleName, "***** Using Limelight ColorBlob Vision.");
                    robot.vision.setLimelightPipeline(Vision.LimelightPipelineType.ColorBlob);
                    visionExpiredTime = null;
                    sm.setState(State.FIND_OBJ);
                }
                else
                {
                    tracer.traceInfo(moduleName, "***** Using ColorBlob Vision but Vision is not enabled.");
                    sm.setState(State.DONE);
                }
                break;

            case FIND_OBJ:
                // Use vision to determine the appropriate object location.
                TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> object =
                    robot.vision.getLimelightDetectedObject(
                        FtcLimelightVision.ResultType.Python,
                        taskParams.alliance == FtcAuto.Alliance.Red? LEDIndicator.RED_BLOB: LEDIndicator.BLUE_BLOB,
                        null, -1);
                if (object != null)
                {
                    objPose = object.detectedObj.getObjectPose();
                    tracer.traceInfo(
                        moduleName, "***** Vision found object: objPose=" + objPose);
                    sm.setState(State.PICKUP_OBJ);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find object, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** No object found.");
                    if (robot.ledIndicator != null)
                    {
                        // Indicate we timed out and found nothing.
                        robot.ledIndicator.setStatusPattern(LEDIndicator.NOT_FOUND, true);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case PICKUP_OBJ:
                TrcEvent ppEvent = null;
                robot.intake.autoIntake(owner, pickupEvent, 0.0);
                tracer.traceInfo(moduleName, "***** AutoIntakeForward");
                if (objPose != null && robot.robotBase != null)
                {
                    ppEvent = driveEvent;
                    robot.robotBase.purePursuitDrive.start(owner, driveEvent, 0.0, true, null, objPose);
                    tracer.traceInfo(moduleName, "***** Drive to object at " + objPose);
                }
                sm.waitForEvents(State.DONE, false, pickupEvent, ppEvent);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickup
