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

import teamcode.Robot;
import teamcode.subsystems.LEDIndicator;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
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
        boolean useVision;

        TaskParams(boolean useVision)
        {
            this.useVision = useVision;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "useVision=" + useVision;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent pickupEvent;
    private final TrcEvent driveEvent;

    private boolean useVision = false;
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
     * @param useVision specifies true to use Vision, false otherwise.
     */
    public void autoPickup(String owner, TrcEvent completionEvent, boolean useVision)
    {
        TaskParams taskParams = new TaskParams(useVision);
        tracer.traceInfo(
            moduleName,
            "autoPickup(owner=" + owner + ", event=" + completionEvent + ", taskParams=(" + taskParams + "))");
        this.useVision = useVision;
        startAutoTask(owner, State.START, taskParams, completionEvent);
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
            if (useVision && robot.robotDrive != null)
            {
                success = robot.robotDrive.driveBase.acquireExclusiveAccess(owner);
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
                (useVision && robot.robotDrive != null?
                    ("\n\tdriveBase=" + ownershipMgr.getOwner(robot.robotDrive.driveBase)): ""));
            robot.intake.releaseExclusiveAccess(owner);
            if (useVision && robot.robotDrive != null)
            {
                robot.robotDrive.driveBase.releaseExclusiveAccess(owner);
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
        if (useVision && robot.robotDrive != null)
        {
            robot.robotDrive.cancel();
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
                objPose = null;
                if (!taskParams.useVision)
                {
                    // Not using vision, just turn on intake and let the driver plow towards the object to pick it up
                    // manually.
                    tracer.traceInfo(moduleName, "***** Not using Vision, manual pickup.");
                    sm.setState(State.PICKUP_OBJ);
                }
                else if (robot.vision != null && robot.vision.redBlobVision != null)
                {
                    tracer.traceInfo(moduleName, "***** Using ColorBlob Vision.");
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
                // Use vision to determine the appropriate AprilTag location.
                TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> object =
                    robot.vision.redBlobVision.getBestDetectedTargetInfo(
                        null, null, 0.0, robot.robotInfo.webCam1.camZOffset);
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
                    if (robot.ledIndicator1 != null)
                    {
                        // Indicate we timed out and found nothing.
                        robot.ledIndicator1.setDetectedPattern(LEDIndicator.FOUND_NOTHING);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case PICKUP_OBJ:
                robot.intake.autoIntake(owner, pickupEvent, 0.0);
                sm.addEvent(pickupEvent);
                tracer.traceInfo(moduleName, "***** AutoIntakeForward");
                if (objPose != null && robot.robotDrive != null)
                {
                    robot.robotDrive.purePursuitDrive.start(owner, driveEvent, 0.0, true, objPose);
                    sm.addEvent(driveEvent);
                    tracer.traceInfo(moduleName, "***** Drive to object at " + objPose);
                }
                sm.waitForEvents(State.DONE, false);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickup
