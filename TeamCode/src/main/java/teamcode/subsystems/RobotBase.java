/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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
import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcPinpointOdometry;
import ftclib.sensor.FtcSparkFunOtos;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class RobotBase
{
    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    public static class VisionOnlyParams extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            webCam1 = new Vision.FrontCamParams();
            webCam2 = new Vision.BackCamParams();
            limelight = new Vision.LimelightParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialParams extends FtcRobotDrive.RobotInfo
    {
        public DifferentialParams()
        {
            robotName = "DifferentialRobot";
            // Robot Dimensions (measured from CAD model if possible)
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = FtcMotorActuator.MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor"};
            driveMotorInverted = new boolean[] {true, false};
            odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            // Drive Motor Odometry
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 80.0;        // inches/sec
            robotMaxAcceleration  = 350.0;  // inches/sec2
            robotMaxTurnRate = 80.0;        // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 2.0;
            turnPidTolerance = 2.0;
            yDrivePidCoeffs = new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/profiledMaxVelocity, 0.0);
        }   //DifferentialParams
    }   //class DifferentialParams

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public MecanumParams()
        {
            robotName = "MecanumRobot";
            // Robot Dimensions
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = FtcMotorActuator.MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};
            xOdWheelIndices = new int[] {0};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};
            yOdWheelIndices = new int[] {1, 2};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, 144.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {-12.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                        .setPodOffsets(-144.0, -168.0)
                        .setEncoderResolution(ODWHEEL_CPR / Math.PI * ODWHEEL_DIAMETER)
                        .setEncodersInverted(false, false);
                    absoluteOdometry = new FtcPinpointOdometry("pinpointOdo", ppOdoConfig);
                    headingWrapRangeLow = -180.0;
                    headingWrapRangeHigh = 180.0;
                }
                else if (RobotParams.Preferences.useSparkfunOTOS)
                {
                    FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                        .setOffset(0.0, -12.0 * TrcUtil.INCHES_PER_MM, 0.0)
                        .setScale(1.0, 1.0);
                    absoluteOdometry = new FtcSparkFunOtos("sparkfunOtos", otosConfig);
                }
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry
            xDrivePosScale = 1.0;   // in/count
            yDrivePosScale = 1.0;   // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 80.0;        // inches/sec
            robotMaxAcceleration  = 350.0;  // inches/sec2
            robotMaxTurnRate = 80.0;        // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new TrcPidController.PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new Vision.FrontCamParams();
            webCam2 = new Vision.BackCamParams();
            limelight = new Vision.LimelightParams();
            // Miscellaneous
            indicatorName = "blinkin";
        }   //MecanumParams
    }   //class MecanumParams

    /**
     * This class contains the Swerve Drive Base Parameters.
     */
    public static class SwerveParams extends FtcSwerveDrive.SwerveInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public SwerveParams()
        {
            robotName = "SwerveRobot";
            // Robot Dimensions
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = FtcMotorActuator.MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, 144.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {-12.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                        .setPodOffsets(-144.0, -168.0)
                        .setEncoderResolution(ODWHEEL_CPR / Math.PI * ODWHEEL_DIAMETER)
                        .setEncodersInverted(false, false);
                    absoluteOdometry = new FtcPinpointOdometry("pinpointOdo", ppOdoConfig);
                    headingWrapRangeLow = -180.0;
                    headingWrapRangeHigh = 180.0;
                }
                else if (RobotParams.Preferences.useSparkfunOTOS)
                {
                    FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                        .setOffset(0.0, -12.0 * TrcUtil.INCHES_PER_MM, 0.0)
                        .setScale(1.0, 1.0);
                    absoluteOdometry = new FtcSparkFunOtos("sparkfunOtos", otosConfig);
                }
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry
            xDrivePosScale = 1.0;   // in/count
            yDrivePosScale = 1.0;   // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 80.0;        // inches/sec
            robotMaxAcceleration  = 350.0;  // inches/sec2
            robotMaxTurnRate = 80.0;        // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new TrcPidController.PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = null;
            turnPidCoeffs = new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new Vision.FrontCamParams();
            webCam2 = new Vision.BackCamParams();
            limelight = new Vision.LimelightParams();
            // Miscellaneous
            indicatorName = "blinkin";
            // Steer Encoders
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.474812, 0.467663, 0.541338, 0.545340};
            steerZerosFilePath = RobotParams.Robot.STEER_ZERO_CAL_FILE;
            // Steer Motors
            steerMotorType = FtcMotorActuator.MotorType.CRServo;
            steerMotorNames = new String[] {"lfSteerServo", "rfSteerServo", "lbSteerServo", "rbSteerServo"};
            steerMotorInverted = new boolean[] {true, true, true, true};
            steerMotorPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 1.0;
            // Swerve Modules
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
        }   //SwerveParams
    }   //class SwerveParams

    private final FtcRobotDrive.RobotInfo robotInfo;
    private final FtcRobotDrive robotDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotBase()
    {
        switch (RobotParams.Preferences.robotType)
        {
            case VisionOnly:
                robotInfo = new VisionOnlyParams();
                robotDrive = null;
                break;

            case DifferentialRobot:
                robotInfo = new DifferentialParams();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcDifferentialDrive(robotInfo): null;
                break;

            case MecanumRobot:
                robotInfo = new MecanumParams();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcMecanumDrive(robotInfo): null;
                break;

            case SwerveRobot:
                robotInfo = new SwerveParams();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcSwerveDrive((SwerveParams) robotInfo): null;
                break;

            default:
                robotInfo = null;
                robotDrive = null;
                break;
        }
    }   //RobotBase

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
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FtcRobotDrive getRobotDrive()
    {
        return robotDrive;
    }   //getRobotDrive

}   //class RobotBase
