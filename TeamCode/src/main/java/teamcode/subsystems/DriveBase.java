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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import ftclib.drivebase.FtcDifferentialBase;
import ftclib.drivebase.FtcMecanumBase;
import ftclib.drivebase.FtcRobotBase;
import ftclib.drivebase.FtcSwerveBase;
import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.GoBildaPinpointDriver;
import teamcode.Dashboard;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.drivebase.TrcSwerveDrive;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class DriveBase extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "DriveBase";
    private static final boolean NEED_ZERO_CAL = false;

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // Generic Swerve Drive Base Robot
        SwerveRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // This is useful for developing Vision code where all you need is a Robot Controller and camera.
        VisionOnly
    }   //enum RobotType

    /**
     * This class contains the Swerve Robot Parameters.
     */
    public static class SwerveRobotInfo extends FtcSwerveBase.SwerveInfo
    {
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(0.0054, 0.0, 0.00039, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.25)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(30.0, 150.0, 150.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(1.0, true));

        public SwerveRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.SwerveRobot.toString(), RobotParams.Robot.ROBOT_WIDTH, RobotParams.Robot.ROBOT_LENGTH,
                    336.0*TrcUtil.INCHES_PER_MM, 336.0*TrcUtil.INCHES_PER_MM)
                .setDriveMotorInfo(
                    FtcMotorActuator.MotorType.DcMotor,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new boolean[] {true, false, true, false})
                    .setPinpointOdometry(
                        "pinpointOdo", 0.0, -24.0 * 8, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                        true, false, -180.0, 180.0)
                    .setPidStallDetectionEnabled(true)
                    .setPidDriveParams(false)
                    .setPurePursuitDriveParams(6.0, true, false)
                    .setVisionInfo(Vision.frontCamInfo, null, Vision.limelightInfo)
                    .setIndicators(LEDIndicator.STATUS_LED_NAME, LEDIndicator.COLOR_BLOB_LED_NAME);
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new boolean[] {false, false, false, false},
                    new double[] {0.7333581066135926, 0.244980314089807, 0.1889143994691428, 0.49038870456389433},
                    RobotParams.Robot.steerZeroCalFile)
                .setSteerMotorInfo(
                    FtcMotorActuator.MotorType.CRServo,
                    new String[] {"flSteerServo", "frSteerServo", "blSteerServo", "brSteerServo"},
                    new boolean[] {false, false, false, false})
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //SwerveRobotInfo
    }   //class SwerveRobotInfo

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumRobotInfo extends FtcRobotBase.RobotInfo
    {
        private static final TrcPidController.PidCoefficients xDrivePidCoeffs =
            new TrcPidController.PidCoefficients(0.06, 0.0, 0.0001, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients yDrivePidCoeffs =
            new TrcPidController.PidCoefficients(0.025, 0.02, 0.003, 0.0, 5.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.04, 0.0, 0.002, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(xDrivePidCoeffs, 1.0)
            .setYPidParams(yDrivePidCoeffs, 1.0)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(30.0, 150.0, 150.0);

        public MecanumRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.MecanumRobot.toString(), RobotParams.Robot.ROBOT_WIDTH, RobotParams.Robot.ROBOT_LENGTH,
                    336.0*TrcUtil.INCHES_PER_MM, 336.0*TrcUtil.INCHES_PER_MM)
                .setDriveMotorInfo(
                    FtcMotorActuator.MotorType.DcMotor,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new boolean[] {true, false, true, false})
                .setPinpointOdometry(
                    "pinpointOdo", 0.0, -24.0 * 8, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                    true, true, -180.0, 180.0)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(6.0, true, false)
                .setVisionInfo(Vision.frontCamInfo, null, Vision.limelightInfo)
                .setIndicators(
                    LEDIndicator.STATUS_LED_NAME, LEDIndicator.COLOR_BLOB_LED_NAME);
        }   //MecanumRobotInfo
    }   //class MecanumRobotInfo

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialRobotInfo extends FtcRobotBase.RobotInfo
    {
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(2.0, 2.0)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.25)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(30.0, 150.0, 150.0);

        public DifferentialRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.DifferentialRobot.toString(), RobotParams.Robot.ROBOT_WIDTH, RobotParams.Robot.ROBOT_LENGTH,
                    336.0*TrcUtil.INCHES_PER_MM, 336.0*TrcUtil.INCHES_PER_MM)
                .setIMUInfo(
                    "imu",
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
                .setDriveMotorInfo(
                    FtcMotorActuator.MotorType.DcMotor,
                    new String[] {"flDriveMotor", "frDriveMotor"},
                    new boolean[] {true, false})
                .setMotorOdometry(0.02166184604662450653409090909091)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(6.0, true, false);
        }   //DifferentialRobotInfo
    }   //class DifferentialRobotInfo

    /**
     * This class contains the VisionOnly Parameters. This is for tuning vision with only the Control Hub and no
     * robot.
     */
    public static class VisionOnlyInfo extends FtcRobotBase.RobotInfo
    {
        public VisionOnlyInfo()
        {
            this.setRobotInfo(RobotType.VisionOnly.toString())
                .setVisionInfo(Vision.frontCamInfo, Vision.backCamInfo, Vision.limelightInfo);
        }   //VisionOnlyInfo
    }   //class VisionOnlyInfo

    private final Robot robot;
    private final FtcDashboard dashboard;
    private final FtcRobotBase.RobotInfo robotInfo;
    private final FtcRobotBase robotBase;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access other subsystems if necessary.
     */
    public DriveBase(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.robot = robot;
        dashboard = FtcDashboard.getInstance();
        switch (RobotParams.Preferences.robotType)
        {
            case SwerveRobot:
                robotInfo = new SwerveRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FtcSwerveBase((SwerveRobotInfo) robotInfo): null;
                break;

            case MecanumRobot:
                robotInfo = new MecanumRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FtcMecanumBase(robotInfo): null;
                break;

            case DifferentialRobot:
                robotInfo = new DifferentialRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FtcDifferentialBase(robotInfo): null;
                break;

            case VisionOnly:
                robotInfo = new VisionOnlyInfo();
                robotBase = null;
                break;

            default:
                robotInfo = null;
                robotBase = null;
                break;
        }
    }   //DriveBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FtcRobotBase.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FtcRobotBase getRobotBase()
    {
        return robotBase;
    }   //getRobotBase

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        if (robotBase != null)
        {
            robotBase.cancel();
        }
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
        // DriveBase does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // DriveBase does not support resetState.
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
        if (robotBase.driveBase.supportsHolonomicDrive())
        {
            robotBase.driveBase.holonomicDrive(
                null, inputs[0], inputs[1], inputs[2], robotBase.driveBase.getDriveGyroAngle());
        }
        else
        {
            robotBase.driveBase.arcadeDrive(inputs[1], inputs[2]);
        }

        if (dashboard.isDashboardUpdateEnabled() && RobotParams.Preferences.showDriveBaseStatus)
        {
            dashboard.displayPrintf(
                14, "RobotDrive: Power=(x=%.2f,y=%.2f,rot=%.2f),Mode:%s",
                inputs[0], inputs[1], inputs[2], robotBase.driveBase.getDriveOrientation());
        }
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
                if (robotBase.driveBase.isGyroAssistEnabled())
                {
                    robotBase.driveBase.setGyroAssistEnabled(null);
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Disabling GyroAssist.");
                }
                else
                {
                    robotBase.driveBase.setGyroAssistEnabled(robotBase.purePursuitDrive.getTurnPidCtrl());
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Enabling GyroAssist.");
                }
            }
            else if (robotBase.driveBase.supportsHolonomicDrive())
            {
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (robotBase.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.Field)
                {
                    setDriveOrientation(TrcDriveBase.DriveOrientation.Field);
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Enabling FIELD mode.");
                }
                else
                {
                    setDriveOrientation(TrcDriveBase.DriveOrientation.Robot);
                    robot.globalTracer.traceInfo(instanceName, ">>>>> Enabling ROBOT mode.");
                }
            }
        }
    }   //subsystemAction

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        robot.globalTracer.traceInfo(instanceName, "driveOrientation=" + orientation);
        robotBase.driveBase.setDriveOrientation(orientation, false);

        if (orientation == TrcDriveBase.DriveOrientation.Field)
        {
            robot.robotBase.driveBase.setFieldForwardHeading(
                Dashboard.DashboardParams.alliance == FtcAuto.Alliance.Red? 0.0: 180.0);
        }

        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setDriveOrientation(orientation);
        }
    }   //setDriveOrientation

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
        if (robotBase == null)
        {
            return lineNum;
        }

        if (slowLoop && RobotParams.Preferences.showDriveBaseStatus)
        {
            dashboard.displayPrintf(lineNum++, "Robot: %s", robotBase.driveBase.getFieldPosition());
            dashboard.displayPrintf(
                lineNum++, "DriveEnc: fl=%.0f,fr=%.0f,bl=%.0f,br=%.0f",
                robotBase.driveMotors[MotorIndex.FrontLeft.value].getPosition(),
                robotBase.driveMotors[MotorIndex.FrontRight.value].getPosition(),
                robotBase.driveMotors[MotorIndex.BackLeft.value].getPosition(),
                robotBase.driveMotors[MotorIndex.BackRight.value].getPosition());

            if (robotBase instanceof FtcSwerveBase)
            {
                FtcSwerveBase swerveDrive = (FtcSwerveBase) robotBase;
                dashboard.displayPrintf(
                    lineNum++, "SteerEnc: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f",
                    swerveDrive.steerEncoders[MotorIndex.FrontLeft.value].getScaledPosition(),
                    swerveDrive.steerEncoders[MotorIndex.FrontRight.value].getScaledPosition(),
                    swerveDrive.steerEncoders[MotorIndex.BackLeft.value].getScaledPosition(),
                    swerveDrive.steerEncoders[MotorIndex.BackRight.value].getScaledPosition());
                dashboard.displayPrintf(
                    lineNum++, "SteerRaw: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f",
                    swerveDrive.steerEncoders[MotorIndex.FrontLeft.value].getRawPosition(),
                    swerveDrive.steerEncoders[MotorIndex.FrontRight.value].getRawPosition(),
                    swerveDrive.steerEncoders[MotorIndex.BackLeft.value].getRawPosition(),
                    swerveDrive.steerEncoders[MotorIndex.BackRight.value].getRawPosition());
            }

            if (robotBase.gyro != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "Gyro(x,y,z): Heading=(%.1f,%.1f,%.1f), Rate=(%.3f,%.3f,%.3f)",
                    robotBase.gyro.getXHeading().value, robotBase.gyro.getYHeading().value,
                    robotBase.gyro.getZHeading().value, robotBase.gyro.getXRotationRate().value,
                    robotBase.gyro.getYRotationRate().value,
                    robotBase.gyro.getZRotationRate().value);
            }

            if (RobotParams.Preferences.showPidDrive)
            {
                TrcPidController xPidCtrl = robotBase.pidDrive.getXPidCtrl();
                if (xPidCtrl != null)
                {
                    xPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                }
                robotBase.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                lineNum += 2;
                robotBase.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                lineNum += 2;
            }
        }

        if (RobotParams.Preferences.showDriveBaseGraph)
        {
            for (TrcMotor motor : robotBase.driveMotors)
            {
                dashboard.putNumber(motor.getName() + ".Velocity", motor.getVelocity());
                dashboard.putNumber(motor.getName() + ".TargetVel", motor.getPidTarget());
            }
            dashboard.putNumber("DriveMotorMaxVel", robotInfo.baseParams.driveMotorMaxVelocity);
            dashboard.putNumber("DriveMotorMinVel", 0.0);

            if (robotBase instanceof FtcSwerveBase)
            {
                FtcSwerveBase swerveDrive = (FtcSwerveBase) robotBase;
                for (TrcMotor motor : swerveDrive.steerMotors)
                {
                    dashboard.putNumber(motor.getName() + ".Angle", motor.getPosition()%360.0);
                    dashboard.putNumber(motor.getName() + ".TargetAngle", motor.getPidTarget()%360.0);
                }
                dashboard.putNumber("SteerMinAngle", 0.0);
                dashboard.putNumber("SteerMaxAngle", 360.0);
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName)
    {
        // DriveBase doesn't support tuning.
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsFromDashboard(String subsystemName)
    {
        // DriveBase doesn't support tuning.
    }   //updateParamsFromDashboard

    /**
     * This method is called to set the next tune target up from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetUp(String subsystemName)
    {
        // DriveBase doesn't support tuning.
    }   //setNextTuneTargetUp

    /**
     * This method is called to set the next tune target down from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetDown(String subsystemName)
    {
        // DriveBase doesn't support tuning.
    }   //setNextTuneTargetDown

}   //class DriveBase
