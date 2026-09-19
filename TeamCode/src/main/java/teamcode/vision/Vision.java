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

package teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import ftclib.driverio.FtcDashboard;
import ftclib.robotcore.FtcOpMode;
import ftclib.vision.FtcEocvColorBlobProcessor;
import ftclib.vision.FtcLimelightVision;
import ftclib.vision.FtcVision;
import ftclib.vision.FtcVisionAprilTag;
import ftclib.vision.FtcVisionEocvColorBlob;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVision;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements AprilTag/Eocv/Limelight Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    private final String moduleName = getClass().getSimpleName();

//    // Lens properties for various cameras.
//    private static final TrcOpenCvDetector.LensInfo logitechC920At640x480 =
//        new TrcOpenCvDetector.LensInfo()
//            .setLensProperties(622.001, 622.001, 319.803, 241.251)
//            .setDistortionCoefficents(0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0);
//    private static final TrcOpenCvDetector.LensInfo logitechC270At640x480 =
//        new TrcOpenCvDetector.LensInfo()
//            .setLensProperties(822.317, 822.317, 319.495, 242.502)
//            .setDistortionCoefficents(-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0);
//    private static final TrcOpenCvDetector.LensInfo lifeCamHD3000At640x480 =
//        new TrcOpenCvDetector.LensInfo()
//            .setLensProperties(678.154, 678.170, 318.135, 228.374)
//            .setDistortionCoefficents(0.154576, -1.19143, 0, 0, 2.06105, 0, 0, 0);

    // Limelight camera properties
    public static final int NUM_LIMELIGHT_PIPELINES = 3;
    public static final TrcVision.CameraInfo limelightInfo = new TrcVision.CameraInfo()
        .setCameraInfo("Limelight3a", 640, 480)
        .setCameraFOV(54.505, 42.239)
        .setCameraPose(0.0, 0.0, 16.361, 0.0, 18.0, 0.0);
    // AprilTag webcam properties
    public static final TrcVision.CameraInfo aprilTagWebcamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("Webcam 1", 640, 480)
        .setCameraPose(-4.25, 5.5, 10.608, -2.0, -32.346629699, 0.0);
//        .setLensProperties(logitechC920At640x480);
    // ColorBlob webcam properties
    public static final TrcVision.CameraInfo colorBlobWebcamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("Webcam 2", 640, 480)
        .setCameraPose(0.0, 2.0, 9.75, 0.0, 15.0, 0.0)
//        .setLensProperties(logitechC920At640x480)
        .setHomographyParams(
            new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                     // Camera Top Left
                639.0, 120.0,                   // Camera Top Right
                0.0, 479.0,                     // Camera Bottom Left
                639.0, 479.0),                  // Camera Bottom Right
            new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - 2.0,  // World Top Left
                11.4375, 44.75 - RobotParams.Robot.ROBOT_LENGTH/2.0 - 2.0,  // World Top Right
                -2.5625, 21.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - 2.0,   // World Bottom Left
                2.5626, 21.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - 2.0));  // World Bottom Right

    public enum ColorBlobType
    {
        None(0),
        Yellow(1),
        Blue(2);

        public final int id;
        ColorBlobType(int id)
        {
            this.id = id;
        }   //ColorBlobType

        public static ColorBlobType fromId(int id)
        {
            for (ColorBlobType type: values())
            {
                if (type.id == id) return type;
            }

            return None;
        }   //fromId
    }   //enum ColorBlobType

    public enum LimelightPipelineType
    {
        AprilTag(0),
        ColorBlob(1),
        NeuralNet(2);

        public final int id;
        LimelightPipelineType(int id)
        {
            this.id = id;
        }   //LimelightPipelineType

        public static LimelightPipelineType fromId(int id)
        {
            for (LimelightPipelineType type: values())
            {
                if (type.id == id) return type;
            }

            return null;
        }   //fromId
    }   //enum LimelightPipelineType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    // HSV Color Space.
    private static final TrcOpenCvColorBlobPipeline.ColorConversion colorConversion =
        TrcOpenCvColorBlobPipeline.ColorConversion.RGBToHSV;
    private static final double[] yellowThresholdsLow = {20.0, 100.0, 60.0};
    private static final double[] yellowThresholdsHigh = {50.0, 255.0, 255.0};
//    private static final double[] red1ThresholdsLow = {0.0, 80.0, 60.0};
//    private static final double[] red1ThresholdsHigh = {20.0, 255.0, 255.0};
//    private static final double[] red2ThresholdsLow = {160.0, 50.0, 100.0};
//    private static final double[] red2ThresholdsHigh = {180.0, 255.0, 255.0};
    private static final double[] blueThresholdsLow = {100.0, 100.0, 100.0};
    private static final double[] blueThresholdsHigh = {140.0, 255.0, 255.0};

    public static final TrcOpenCvColorBlobPipeline.FilterContourParams colorBlobFilterParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(500.0)
            .setMinPerimeter(100.0)
            .setWidthRange(10.0, 1000.0)
            .setHeightRange(10.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.5, 2.5);
    private static final double colorBlobWidth = 3.6;
    private static final double colorBlobHeight = 3.6;
    // Create the pipeline parameters for both yellow, red and blue colorblob here so that Dashboard can access them.
    public static final TrcOpenCvColorBlobPipeline.PipelineParams colorBlobPipelineParams =
        new TrcOpenCvColorBlobPipeline.PipelineParams()
            .setAnnotation(false, false)
            .setColorConversion(colorConversion)
            .addColorThresholds(LEDIndicator.YELLOW_BLOB, true, yellowThresholdsLow, yellowThresholdsHigh)
            .addColorThresholds(LEDIndicator.BLUE_BLOB, true, blueThresholdsLow, blueThresholdsHigh)
            .buildColorThresholdSets()
            .setFilterContourParams(true, colorBlobFilterParams);

    private final TrcDbgTrace tracer;
    private final Robot robot;
    public FtcLimelightVision limelightVision;
    private WebcamName aprilTagWebcam = null;
    public FtcVisionAprilTag aprilTagWebcamVision;
    private AprilTagProcessor aprilTagWebcamProcessor;
    private WebcamName colorBlobWebcam = null;
    public FtcVisionEocvColorBlob colorBlobWebcamVision;
    private FtcEocvColorBlobProcessor colorBlobWebcamProcessor;
    public FtcVision ftcVision;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

        this.tracer = new TrcDbgTrace();
        this.robot = robot;

        for (TrcVision.CameraInfo camInfo: robot.robotInfo.camInfos)
        {
            if (camInfo == limelightInfo && RobotParams.Preferences.useLimelightVision)
            {
                tracer.traceInfo(moduleName, "Starting LimelightVision...");
                limelightVision = new FtcLimelightVision(camInfo, this::getLimelightTargetGroundOffset);
                setLimelightPipeline(LimelightPipelineType.AprilTag);
            }
            else if (camInfo == aprilTagWebcamInfo && RobotParams.Preferences.useWebCam &&
                     RobotParams.Preferences.useWebcamAprilTagVision)
            {
                tracer.traceInfo(moduleName, "Starting WebcamVision for AprilTag...");
                aprilTagWebcam = opMode.hardwareMap.get(WebcamName.class, camInfo.camName);
                FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                    .setDrawTagIdEnabled(true)
                    .setDrawTagOutlineEnabled(true)
                    .setDrawAxesEnabled(false)
                    .setDrawCubeProjectionEnabled(false)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
                aprilTagWebcamVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11);
                aprilTagWebcamProcessor = aprilTagWebcamVision.getVisionProcessor();
                visionProcessorsList.add(aprilTagWebcamProcessor);
            }
            else if (camInfo == colorBlobWebcamInfo && RobotParams.Preferences.useWebCam &&
                     RobotParams.Preferences.useWebcamColorBlobVision)
            {
                TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams = null;

                tracer.traceInfo(moduleName, "Starting WebcamVision for ColorBlob...");
                colorBlobWebcam = opMode.hardwareMap.get(WebcamName.class, camInfo.camName);
                if (RobotParams.Preferences.useSolvePnp)
                {
                    solvePnpParams =
                        new TrcOpenCvColorBlobPipeline.SolvePnpParams().setObjectSize(colorBlobWidth, colorBlobHeight);
                    if (camInfo.lensInfo != null)
                    {
                        solvePnpParams.setSolvePnpParams(camInfo.lensInfo, camInfo.camPose);
                    }
                }

                colorBlobWebcamVision = new FtcVisionEocvColorBlob(
                    "ColorBlobVision", colorBlobPipelineParams, solvePnpParams, camInfo.cameraRect, camInfo.worldRect);
                colorBlobWebcamProcessor = colorBlobWebcamVision.getVisionProcessor();
                visionProcessorsList.add(colorBlobWebcamProcessor);
            }
        }

        if (!visionProcessorsList.isEmpty())
        {
            VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
            visionProcessorsList.toArray(visionProcessors);
            if (RobotParams.Preferences.useWebCam)
            {
                // Use USB webcams.
                ftcVision = new FtcVision(
                    aprilTagWebcam, colorBlobWebcam, aprilTagWebcamInfo.camImageWidth,
                    aprilTagWebcamInfo.camImageHeight, RobotParams.Preferences.showVisionView,
                    RobotParams.Preferences.showVisionStat, visionProcessors);
            }

            // Disable all vision until they are needed.
            for (VisionProcessor processor: visionProcessors)
            {
                ftcVision.setProcessorEnabled(processor, false);
            }
        }

        FtcDashboard.getInstance().addStatusUpdate(moduleName, this::updateStatus);
    }   //Vision

    /**
     * This method closes the vision portal and is normally called at the end of an opmode.
     */
    public void close()
    {
        if (ftcVision != null)
        {
            ftcVision.close();
        }
    }   //close

    /**
     * This method enables/disables FPS meter on the viewport.
     *
     * @param enabled specifies true to enable FPS meter, false to disable.
     */
    public void setFpsMeterEnabled(boolean enabled)
    {
        if (ftcVision != null)
        {
            ftcVision.setFpsMeterEnabled(enabled);
        }
    }   //setFpsMeterEnabled

    /**
     * This method returns the AprilTag webcam.
     *
     * @return AprilTag webcam.
     */
    public WebcamName getAprilTagWebcam()
    {
        return aprilTagWebcam;
    }   //getAprilTagWebcam

    /**
     * This method returns the ColorBlob webcam.
     *
     * @return ColorBlob webcam.
     */
    public WebcamName getColorBlobWebcam()
    {
        return colorBlobWebcam;
    }   //getColorBlobWebcam

    /**
     * This method returns the active camera if we have two webcams.
     *
     * @return active camera.
     */
    public WebcamName getActiveWebcam()
    {
        return ftcVision.getActiveWebcam();
    }   //getActiveWebcam

    /**
     * This method sets the active webcam.
     *
     * @param webcam specifies the webcam to be set as active.
     */
    public void setActiveWebcam(WebcamName webcam)
    {
        ftcVision.setActiveWebcam(webcam);
    }   //setActiveWebcam

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     *
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = ftcVision.getExposureSetting();
        long currExposure = ftcVision.getCurrentExposure();
        int[] gainSetting = ftcVision.getGainSetting();
        int currGain = ftcVision.getCurrentGain();

        if (exposureSetting != null && gainSetting != null)
        {
            robot.dashboard.displayPrintf(
                lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
                currExposure, exposureSetting[0], exposureSetting[1], currGain, gainSetting[0], gainSetting[1]);
        }
    }   //displayExposureSettings

    /**
     * This method enables/disables Limelight vision for the specified pipeline.
     *
     * @param pipelineType specifies the limelight pipeline type to be selected, ignore if disabled.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setLimelightVisionEnabled(LimelightPipelineType pipelineType, boolean enabled)
    {
        if (limelightVision != null && enabled ^ limelightVision.isVisionEnabled())
        {
            if (enabled)
            {
                setLimelightPipeline(pipelineType);
            }
            limelightVision.setVisionEnabled(enabled);
            tracer.traceInfo(
                moduleName, "Pipeline %s is %s: running=%s",
                pipelineType, enabled? "enabled": "disabled", limelightVision.limelight.isRunning());
        }
    }   //setLimelightVisionEnabled

    /**
     * This method checks if Limelight vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isLimelightVisionEnabled()
    {
        return limelightVision != null && limelightVision.isVisionEnabled();
    }   //isLimelightVisionEnabled

    /**
     * This method sets the Limelight pipeline.
     *
     * @param pipelineType specifies the pipeline type.
     */
    public void setLimelightPipeline(LimelightPipelineType pipelineType)
    {
        if (limelightVision != null && limelightVision.isVisionEnabled())
        {
            limelightVision.setPipeline(pipelineType.id);
            limelightVision.setStatusResultType(
                pipelineType == LimelightPipelineType.AprilTag? FtcLimelightVision.ResultType.Fiducial:
                pipelineType == LimelightPipelineType.ColorBlob? FtcLimelightVision.ResultType.Python: null);
        }
    }   //setLimelightPipeline

    /**
     * This method calls Limelight vision to detect the object.
     *
     * @param resultType specifies the result type to look for.
     * @param matchIds specifies the object ID(s) to match for, null if no matching required.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Limelight object info.
     */
    public TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> getLimelightDetectedObject(
        FtcLimelightVision.ResultType resultType, Object matchIds,
        Comparator<? super TrcVisionTargetInfo<FtcLimelightVision.DetectedObject>> comparator, int lineNum)
    {
        TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> limelightInfo = null;
        LimelightPipelineType pipelineType;

        if (limelightVision != null &&
            (pipelineType = LimelightPipelineType.fromId(limelightVision.getPipeline())) != null)
        {
            String objectName = null;

            limelightInfo = limelightVision.getBestDetectedTargetInfo(resultType, matchIds, comparator);
            if (limelightInfo != null)
            {
                switch (pipelineType)
                {
                    case AprilTag:
                        objectName = (int) limelightInfo.detectedObj.objId == RobotParams.Game.BLUE_APRILTAG_ID?
                            LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setAprilTagPatternsOff();
                            robot.ledIndicator.setStatusPattern(objectName, true);
                        }
                        break;

                    case ColorBlob:
                    case NeuralNet:
                        objectName = (String) limelightInfo.detectedObj.objId;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setColorBlobPatternState(objectName, true);
                        }
                        break;

                    default:
                        break;
                }
            }

            if (lineNum != -1)
            {
                robot.dashboard.displayPrintf(
                    lineNum, "%s(pipeline=%s): %s",
                    objectName, pipelineType, limelightInfo != null? limelightInfo: "Not found.");
            }
        }

        return limelightInfo;
    }   //getLimelightDetectedObject

    /**
     * This method enables/disables the Vision Processor.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setVisionProcessorEnabled(VisionProcessor processor, boolean enabled)
    {
        if (processor != null)
        {
            ftcVision.setProcessorEnabled(processor, enabled);
        }
    }   //setVisionProcessorEnabled

    /**
     * This method checks if the Vision Processor is enabled.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @return true if enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor processor)
    {
        return processor != null && ftcVision.isVisionProcessorEnabled(processor);
    }   //isVisionProcessorEnabled

    /**
     * This method enables/disables AprilTag Webcam Vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setAprilTagWebcamVisionEnabled(boolean enabled)
    {
        setVisionProcessorEnabled(aprilTagWebcamProcessor, enabled);
    }   //setAprilTagWebcamVisionEnabled

    /**
     * This method checks if AprilTag Webcam Vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagWebcamVisionEnabled()
    {
        return isVisionProcessorEnabled(aprilTagWebcamProcessor);
    }   //isAprilTagWebcamVisionEnabled

    /**
     * This method calls AprilTag Webcam Vision to detect the AprilTag object.
     *
     * @param aprilTagIds specifies an array of AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getWebcamDetectedAprilTag(
        int[] aprilTagIds, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
            aprilTagWebcamVision.getBestDetectedTargetInfo(aprilTagIds, null);

        if (aprilTagInfo != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setAprilTagPatternsOff();
            if (aprilTagInfo.detectedObj.aprilTagDetection instanceof AprilTagSingleDetection)
            {
                AprilTagSingleDetection singleDet =
                    (AprilTagSingleDetection) aprilTagInfo.detectedObj.aprilTagDetection;
                robot.ledIndicator.setStatusPattern(
                    singleDet.id == RobotParams.Game.BLUE_APRILTAG_ID?
                        LEDIndicator.BLUE_APRILTAG: LEDIndicator.RED_APRILTAG, true);
            }
            else
            {
                AprilTagClusterDetection clusterDet =
                    (AprilTagClusterDetection) aprilTagInfo.detectedObj.aprilTagDetection;
                robot.ledIndicator.setStatusPattern(clusterDet.metadata.name, true);
            }
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "AprilTag[%s]: %s",
                Arrays.toString(aprilTagIds), aprilTagInfo != null ? aprilTagInfo : "Not found.");
        }

        return aprilTagInfo;
    }   //getWebcamDetectedAprilTag

    /**
     * This method enables/disables ColorBlob Webcam Vision for the specified ColorBlob type.
     *
     * @param colorBlobType specifies the ColorBlob type to be detected, can be null for any type.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setColorBlobWebcamVisionEnabled(ColorBlobType colorBlobType, boolean enabled)
    {
        TrcOpenCvColorBlobPipeline colorBlobPipeline =
            colorBlobWebcamProcessor != null? colorBlobWebcamProcessor.getPipeline(): null;

        if (colorBlobPipeline != null)
        {
            if (colorBlobType == null)
            {
                colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB, enabled);
                colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.BLUE_BLOB, enabled);
            }
            else
            {
                switch (colorBlobType)
                {
                    case Yellow:
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB, enabled);
                        if (enabled)
                        {
                            colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.BLUE_BLOB, false);
                        }
                        break;

                    case Blue:
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.BLUE_BLOB, enabled);
                        if (enabled)
                        {
                            colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB, false);
                        }
                        break;
                }
            }

            if (enabled)
            {
                // Start Dashboard Stream before turning on ColorBlob Processor.
                setDashboardStreamEnabled(colorBlobWebcamProcessor, true);
                setVisionProcessorEnabled(colorBlobWebcamProcessor, true);
            }
            else
            {
                // We are disabling a color threshold set in the ColorBlob pipeline. If all color threshold sets
                // are disabled, disable the ColorBlob vision processor as well.
                if (!colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB) &&
                    !colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.BLUE_BLOB))
                {
                    setVisionProcessorEnabled(colorBlobWebcamProcessor, false);
                    setDashboardStreamEnabled(colorBlobWebcamProcessor, false);
                }
            }
        }
    }   //setColorBlobWebcamVisionEnabled

    /**
     * This method checks if ColorBlob Webcam Vision is enabled for the specified ColorBlob type.
     *
     * @param colorBlobType specifies the ColorBlob type to be detected, can be null for any type.
     * @return true if enabled, false if disabled.
     */
    public boolean isColorBlobWebcamVisionEnabled(ColorBlobType colorBlobType)
    {
        boolean enabled = false;
        TrcOpenCvColorBlobPipeline colorBlobPipeline =
            colorBlobWebcamProcessor != null && isVisionProcessorEnabled(colorBlobWebcamProcessor)?
                colorBlobWebcamProcessor.getPipeline(): null;

        if (colorBlobPipeline != null)
        {
            if (colorBlobType == null)
            {
                enabled = colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB) ||
                    colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.BLUE_BLOB);
            }
            else
            {
                switch (colorBlobType)
                {
                    case Yellow:
                        enabled = colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.YELLOW_BLOB);
                        break;

                    case Blue:
                        enabled = colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.BLUE_BLOB);
                        break;
                }
            }
        }

        return enabled;
    }   //isColorBlobWebcamVisionEnabled

    /**
     * This method calls ColorBlob Webcam Vision to detect the specified ColorBlob.
     *
     * @param colorBlobType specifies the ColorBlob type to be detected, can be null for any type.
     * @param groundOffset specifies the ground offset of the detected sample.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected color blob object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getWebcamDetectedColorBlob(
        ColorBlobType colorBlobType, double groundOffset, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> colorBlobInfo = null;

        if (isColorBlobWebcamVisionEnabled(colorBlobType))
        {
            colorBlobInfo = colorBlobWebcamVision == null? null:
                colorBlobWebcamVision.getBestDetectedTargetInfo(
                    this::colorBlobFilter, colorBlobType, this::compareDistanceY, groundOffset, colorBlobWebcamInfo.camPose.z);
        }

        if (colorBlobInfo != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setColorBlobPatternState(colorBlobInfo.detectedObj.label, true);
        }

        if (lineNum != -1)
        {
            if (colorBlobInfo != null)
            {
                robot.dashboard.displayPrintf(lineNum, "%s: %s", colorBlobInfo.detectedObj.label, colorBlobInfo);
            }
            else
            {
                robot.dashboard.displayPrintf(lineNum, "No ColorBlob found.");
            }
        }

        return colorBlobInfo;
    }   //getWebcamDetectedColorBlob

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @param camPose3d specifies the camera pose on the robot, can be null if not provided in which case the returned
     *        pose is the camera field pose.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo, TrcPose3D camPose3d)
    {
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null && aprilTagInfo.detectedObj.aprilTagDetection.robotPose != null)
        {
            Pose3D cameraFieldPose3d = aprilTagInfo.detectedObj.aprilTagDetection.robotPose;
            Position position = cameraFieldPose3d.getPosition();
            double ftcYaw = cameraFieldPose3d.getOrientation().getYaw();

            if (camPose3d != null)
            {
                double robotFtcYawDeg = ftcYaw + camPose3d.yaw;
                double robotFtcYawRad = Math.toRadians(robotFtcYawDeg);
                double cosYaw = Math.cos(robotFtcYawRad);
                double sinYaw = Math.sin(robotFtcYawRad);
                double robotToCamFieldX = camPose3d.x * sinYaw + camPose3d.y * cosYaw;
                double robotToCamFieldY = -camPose3d.x * cosYaw + camPose3d.y * sinYaw;
                double robotFieldX = position.x - robotToCamFieldX;
                double robotFieldY = position.y - robotToCamFieldY;
                double robotTrcHeading = (90.0 - ftcYaw) - camPose3d.yaw;

                robotPose = new TrcPose2D(robotFieldX, robotFieldY, robotTrcHeading);
            }
            else
            {
                // If camPose3d is null, return the raw camera field pose translated to TrcLib convention
                robotPose = new TrcPose2D(position.x, position.y, 90.0 - ftcYaw);
            }

            tracer.traceInfo(
                moduleName,
                "robotPose3d=" + aprilTagInfo.detectedObj.aprilTagDetection.robotPose +
                    ", camPose3d=" + camPose3d +
                    ", trcRobotPose=" + robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @param camPose3d specifies the camera pose on the robot, can be null if not provided in which case the returned
     *        pose is the camera field pose.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcPose3D camPose3d)
    {
        TrcPose2D robotPose = null;

        if (isAprilTagWebcamVisionEnabled())
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getWebcamDetectedAprilTag(null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo, camPose3d);
            }
        }
        else if (isLimelightVisionEnabled() &&
            LimelightPipelineType.fromId(limelightVision.getPipeline()) == LimelightPipelineType.AprilTag)
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                getLimelightDetectedObject(FtcLimelightVision.ResultType.Fiducial, null, null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = aprilTagInfo.detectedObj.robotPose;
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method enables/disables the Dashboard Streaming of the specified ColorBlob processor.
     *
     * @param processor specifies the ColorBlob processor to have the Dashboard stream enabled/disabled.
     * @param enabled specifies true to enable stream, false to disable.
     */
    public void setDashboardStreamEnabled(FtcEocvColorBlobProcessor processor, boolean enabled)
    {
        if (RobotParams.Preferences.streamWebcamToDashboard)
        {
            if (enabled)
            {
                processor.enableDashboardStream();
            }
            else
            {
                processor.disableDashboardStream();
            }
        }
    }   //setDashboardStreamEnabled

    /**
     * This method returns the Limelight target Z offset from ground.
     *
     * @param resultType specifies the detected object result type.
     * @return target ground offset.
     */
    private double getLimelightTargetGroundOffset(FtcLimelightVision.ResultType resultType)
    {
        double offset;

        switch (resultType)
        {
            case Fiducial:
                offset = 29.5;
                break;

            case Python:
            default:
                offset = 0.0;
                break;
        }

        return offset;
    }   //getLimelightTargetGroundOffset

    /**
     * This method is called by ColorBlob Webcam Vision to validate if the detected ColorBlob matches expectation
     * for filtering.
     *
     * @param colorBlobInfo specifies the detected ColorBlob info, can be null for any type.
     * @param context specifies the expected color blob type.
     * @return true if it matches expectation, false otherwise.
     */
    public boolean colorBlobFilter(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> colorBlobInfo, Object context)
    {
        ColorBlobType colorBlobType = (ColorBlobType) context;
        boolean match = false;

        if (colorBlobType == null)
        {
            match = colorBlobInfo.detectedObj.label.equals(LEDIndicator.YELLOW_BLOB) ||
                    colorBlobInfo.detectedObj.label.equals(LEDIndicator.BLUE_BLOB);
        }
        else
        {
            switch (colorBlobType)
            {
                case Yellow:
                    match = colorBlobInfo.detectedObj.label.equals(LEDIndicator.YELLOW_BLOB);
                    break;

                case Blue:
                    match = colorBlobInfo.detectedObj.label.equals(LEDIndicator.BLUE_BLOB);
                    break;
            }
        }

        return match;
    }   //colorBlobFilter

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    public int compareDistanceY(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((b.objPose.y - a.objPose.y)*100);
    }   //compareDistanceY

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (RobotParams.Preferences.showVisionStatus)
        {
            if (slowLoop)
            {
                if (limelightVision != null)
                {
                    lineNum = limelightVision.updateStatus(lineNum);
                }

                if (aprilTagWebcamVision != null)
                {
                    lineNum = aprilTagWebcamVision.updateStatus(lineNum);
                }

                if (colorBlobWebcamVision != null)
                {
                    lineNum = colorBlobWebcamVision.updateStatus(lineNum);
                }
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision
