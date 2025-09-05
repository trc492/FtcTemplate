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

package teamcode.autocommands;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoStartPosLeft implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdAutoStartPosLeft.class.getSimpleName();

    private enum State
    {
        START,
        SCORE_PRELOAD,
        GOTO_RING_POS,
        PICKUP_RING,
        GOTO_START_POS,
        SCORE_RING,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private TrcPose2D startPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoStartPosLeft(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAutoStartPosLeft

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     * The hypothetical autonomous is described as follows:
     * - The robot starts at the location where an AprilTag is right in front of it.
     * - The robot is preloaded with one ring and it will shoot the ring into the goal right above the ArpilTag.
     * - The robot will then go to a location where there is a second ring on the ground.
     * - The robot will pick up the ring on the ground.
     * - The robot will go back to the location where it scored the preloaded ring.
     * - The robot will shoot the picked up ring into the goal.
     * - Done.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    startPose = robot.robotDrive.driveBase.getFieldPosition();
                    // Retrieve auto choice options.
                    // Do delay if necessary.
                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    break;

                case SCORE_PRELOAD:
                    if (autoChoices.scorePreload)
                    {
                        robot.autoShootTask.autoShoot(null, event, autoChoices.useVision, (int[]) null);
                        sm.waitForSingleEvent(event, State.GOTO_RING_POS);
                    }
                    else
                    {
                        sm.setState(State.GOTO_RING_POS);
                    }
                    break;

                case GOTO_RING_POS:
                    robot.robotDrive.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.adjustPoseByAlliance(RobotParams.Game.BLUE_PICKUP_RING_POSE, autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.PICKUP_RING);
                    break;

                case PICKUP_RING:
                    robot.autoPickupTask.autoPickup(null, event, autoChoices.useVision);
                    sm.waitForSingleEvent(event, State.GOTO_START_POS);
                    break;

                case GOTO_START_POS:
                    robot.robotDrive.purePursuitDrive.start(
                        null, event, 0.0, false, startPose);
                    sm.waitForSingleEvent(event, State.SCORE_RING);
                    break;

                case SCORE_RING:
                    robot.autoShootTask.autoShoot(null, event, autoChoices.useVision, (int[]) null);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoStartPosLeft
