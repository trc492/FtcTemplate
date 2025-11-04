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

import ftclib.drivebase.FtcDifferentialDrive;
import ftclib.drivebase.FtcMecanumDrive;
import ftclib.drivebase.FtcRobotDrive;
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.GoBildaPinpointDriver;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class BaseDrive extends TrcSubsystem
{
    private static final String moduleName = BaseDrive.class.getSimpleName();

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Robot Controller and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    /**
     * This class contains the VisionOnly Parameters. This is for tuning vision with only the Control Hub and no
     * robot.
     */
    public static class VisionOnlyInfo extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyInfo()
        {
            this.setRobotInfo("VisionOnly")
                .setVisionInfo(Vision.frontCamParams, Vision.backCamParams, Vision.limelightParams);
        }   //VisionOnlyInfo
    }   //class VisionOnlyInfo

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialRobotInfo extends FtcRobotDrive.RobotInfo
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
            .setDriveCharacteristics(30.0, 150.0, 150.0,  15.0);

        public DifferentialRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.MecanumRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
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
     * This class contains the Mecanum Drive Base Parameters.
     */
    public static class MecanumRobotInfo extends FtcRobotDrive.RobotInfo
    {
        private static final double DRIVE_MOTOR_MAX_VEL = 1000.0;
        private static final double DRIVE_MOTOR_VEL_PID_TOLERANCE = 10.0;

        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.0001, 0.0, 0.0, 0.5);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                DRIVE_MOTOR_MAX_VEL, driveMotorVelPidCoeffs, DRIVE_MOTOR_VEL_PID_TOLERANCE, true)
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.25)
            .setVelocityPidParams(velPidCoeffs)
            .setDriveCharacteristics(30.0, 150.0, 150.0,  15.0);

        public MecanumRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.MecanumRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
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
                .setVisionInfo(Vision.frontCamParams, null, Vision.limelightParams)
                .setIndicators(LEDIndicator.STATUS_LED_NAME, LEDIndicator.COLOR_BLOB_LED_NAME);
        }   //MecanumRobotInfo
    }   //class MecanumRobotInfo

    /**
     * This class contains the Swerve Drive Base Parameters.
     */
    public static class SwerveRobotInfo extends FtcSwerveDrive.SwerveInfo
    {
        private static final double DRIVE_MOTOR_MAX_VEL = 1000.0;
        private static final double DRIVE_MOTOR_VEL_PID_TOLERANCE = 10.0;

        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.0001, 0.0, 0.0, 0.5);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(0.0054, 0.0, 0.00039, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                DRIVE_MOTOR_MAX_VEL, driveMotorVelPidCoeffs, DRIVE_MOTOR_VEL_PID_TOLERANCE, true)
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.25)
            .setVelocityPidParams(velPidCoeffs)
            .setDriveCharacteristics(30.0, 150.0, 150.0,  15.0);
        public static TrcSwerveDriveBase.SwerveParams swerveParams = new TrcSwerveDriveBase.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(1.0, true));

        public SwerveRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.SwerveRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
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
                .setVisionInfo(Vision.frontCamParams, null, Vision.limelightParams)
                .setIndicators(LEDIndicator.STATUS_LED_NAME, LEDIndicator.COLOR_BLOB_LED_NAME);
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new boolean[] {false, false, false, false},
                    new double[] {0.7333581066135926, 0.244980314089807, 0.1889143994691428, 0.49038870456389433},
                    RobotParams.Robot.STEER_ZERO_CAL_FILE)
                .setSteerMotorInfo(
                    FtcMotorActuator.MotorType.CRServo,
                    new String[] {"flSteerServo", "frSteerServo", "blSteerServo", "brSteerServo"},
                    new boolean[] {false, false, false, false})
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //SwerveRobotInfo
    }   //class SwerveRobotInfo

    private final FtcDashboard dashboard;
    private final FtcRobotDrive.RobotInfo robotInfo;
    private final FtcRobotDrive robotDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public BaseDrive()
    {
        super(RobotParams.Preferences.robotType.toString(), false);
        dashboard = FtcDashboard.getInstance();
        switch (RobotParams.Preferences.robotType)
        {
            case VisionOnly:
                robotInfo = new VisionOnlyInfo();
                robotDrive = null;
                break;

            case DifferentialRobot:
                robotInfo = new DifferentialRobotInfo();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcDifferentialDrive(robotInfo): null;
                break;

            case MecanumRobot:
                robotInfo = new MecanumRobotInfo();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcMecanumDrive(robotInfo): null;
                break;

            case SwerveRobot:
                robotInfo = new SwerveRobotInfo();
                robotDrive =
                    RobotParams.Preferences.useDriveBase? new FtcSwerveDrive((SwerveRobotInfo) robotInfo): null;
                break;

            default:
                robotInfo = null;
                robotDrive = null;
                break;
        }
    }   //BaseDrive

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FtcRobotDrive.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created BaseDrive object.
     *
     * @return created robot drive.
     */
    public FtcRobotDrive getRobotDrive()
    {
        return robotDrive;
    }   //getRobotDrive

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        if (robotDrive != null)
        {
            robotDrive.cancel();
        }
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // BaseDrive does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // BaseDrive does not support resetState.
    }   //resetState

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
        if (robotDrive == null)
        {
            return lineNum;
        }

        if (RobotParams.Preferences.showDriveBaseStatus)
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(lineNum++, "Robot: %s", robotDrive.driveBase.getFieldPosition());
                dashboard.displayPrintf(
                    lineNum++, "DriveEnc: fl=%.0f,fr=%.0f,bl=%.0f,br=%.0f",
                    robotDrive.driveMotors[FtcRobotDrive.INDEX_FRONT_LEFT].getPosition(),
                    robotDrive.driveMotors[FtcRobotDrive.INDEX_FRONT_RIGHT].getPosition(),
                    robotDrive.driveMotors[FtcRobotDrive.INDEX_BACK_LEFT].getPosition(),
                    robotDrive.driveMotors[FtcRobotDrive.INDEX_BACK_RIGHT].getPosition());

                if (robotDrive instanceof FtcSwerveDrive)
                {
                    FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robotDrive;
                    dashboard.displayPrintf(
                        lineNum++, "SteerEnc: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f",
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_FRONT_LEFT].getScaledPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_FRONT_RIGHT].getScaledPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_BACK_LEFT].getScaledPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_BACK_RIGHT].getScaledPosition());
                    dashboard.displayPrintf(
                        lineNum++, "SteerRaw: fl=%.2f, fr=%.2f, bl=%.2f, br=%.2f",
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_FRONT_LEFT].getRawPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_FRONT_RIGHT].getRawPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_BACK_LEFT].getRawPosition(),
                        swerveDrive.steerEncoders[FtcRobotDrive.INDEX_BACK_RIGHT].getRawPosition());
                }

                if (robotDrive.gyro != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Gyro(x,y,z): Heading=(%.1f,%.1f,%.1f), Rate=(%.3f,%.3f,%.3f)",
                        robotDrive.gyro.getXHeading().value, robotDrive.gyro.getYHeading().value,
                        robotDrive.gyro.getZHeading().value, robotDrive.gyro.getXRotationRate().value,
                        robotDrive.gyro.getYRotationRate().value,
                        robotDrive.gyro.getZRotationRate().value);
                }

                if (RobotParams.Preferences.showPidDrive)
                {
                    TrcPidController xPidCtrl = robotDrive.pidDrive.getXPidCtrl();
                    if (xPidCtrl != null)
                    {
                        xPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                    lineNum += 2;
                    robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                    lineNum += 2;
                }
            }
        }

        if (RobotParams.Preferences.showDriveBaseGraph)
        {
            for (TrcMotor motor : robotDrive.driveMotors)
            {
                dashboard.putNumber(motor.getName() + ".Velocity", motor.getVelocity());
                dashboard.putNumber(motor.getName() + ".TargetVel", motor.getPidTarget());
            }
            dashboard.putNumber("DriveMotorMaxVel", robotInfo.baseParams.driveMotorMaxVelocity);
            dashboard.putNumber("DriveMotorMinVel", 0.0);

            if (robotDrive instanceof FtcSwerveDrive)
            {
                FtcSwerveDrive swerveDrive = (FtcSwerveDrive) robotDrive;
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
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class BaseDrive
