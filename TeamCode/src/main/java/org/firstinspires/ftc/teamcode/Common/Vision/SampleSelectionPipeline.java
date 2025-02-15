package org.firstinspires.ftc.teamcode.Common.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.LinkedList;
import java.util.List;
public class SampleSelectionPipeline {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private int selectedClassIndex = 0;
    private String[] classNames = {"RedSample", "BlueSample", "YellowSample"};
    private static final double BOTTOM_CENTER_X = 0.0;
    private static final double BOTTOM_CENTER_Y = -30.0;

    private WebcamPipeline webcamPipeline;
    private KalmanFilter limelightKalman = new KalmanFilter();
    private KalmanFilter webcamKalman = new KalmanFilter();

    public String selectedSampleColor = "None";  // Track the color of the selected sample

    public SampleSelectionPipeline(Limelight3A limelightInstance, Telemetry telemetry, WebcamPipeline webcamPipeline) {
        this.limelight = limelightInstance;
        this.telemetry = telemetry;
        this.webcamPipeline = webcamPipeline;
    }

    public void init() {
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setSelectedClass(int classIndex) {
        if (classIndex >= 0 && classIndex < classNames.length) {
            selectedClassIndex = classIndex;
            telemetry.addData("Selected Class", classNames[classIndex]);
            telemetry.update();
        }
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        Point bestLimelightTarget = limelightKalman.update(getRawLimelightTarget(result));
        Point bestWebcamTarget = webcamKalman.update(webcamPipeline.getBestTarget());

        Point mergedTarget = mergeTargets(bestLimelightTarget, bestWebcamTarget);

        telemetry.addData("Final Target", mergedTarget);
        telemetry.addData("Selected Sample Color", selectedSampleColor);  // Report the selected color
        telemetry.update();
    }

    private Point mergeTargets(Point limelight, Point webcam) {
        if (limelight == null || webcam == null) {
            return new Point(999, 999);
        }
        if (Math.abs(limelight.x - webcam.x) <= 5 && Math.abs(limelight.y - webcam.y) <= 5) {
            return new Point((limelight.x + webcam.x) / 2, (limelight.y + webcam.y) / 2);
        }
        return new Point(999, 999);
    }

    private Point getRawLimelightTarget(LLResult result) {
        if (result == null || !result.isValid()) return null;
        List<DetectorResult> detections = result.getDetectorResults();

        DetectorResult bestTarget = null;
        double bestDistance = Double.MAX_VALUE;


        String[] targetClasses = {"YellowSample", getAllianceColor()};

        for (DetectorResult detection : detections) {
            for (String className : targetClasses) {
                if (detection.getClassName().equals(className)) {
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();
                    double distance = Math.sqrt(Math.pow(tx - BOTTOM_CENTER_X, 2) + Math.pow(ty - BOTTOM_CENTER_Y, 2));

                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestTarget = detection;
                    }
                }
            }
        }
        return (bestTarget == null) ? null : new Point(bestTarget.getTargetXDegrees(), bestTarget.getTargetYDegrees());
    }


    private String getAllianceColor() {
        if (Globals.ALLIANCE.equals(Color.RED)) {
            return "RedSample";
        } else if (Globals.ALLIANCE.equals(Color.BLUE)) {
            return "BlueSample";
        }
        return "UnknownSample";  // Default case
    }


    public class WebcamPipeline extends OpenCvPipeline {
        private Point bestTarget = null;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            Scalar lowerSecondColor, upperSecondColor;
            if (Globals.ALLIANCE.equals(Color.RED)) {
                lowerSecondColor = new Scalar(0, 100, 100);
                upperSecondColor = new Scalar(10, 255, 255);
            } else if (Globals.ALLIANCE.equals(Color.BLUE)) {
                lowerSecondColor = new Scalar(100, 150, 50);
                upperSecondColor = new Scalar(140, 255, 255);
            } else {
                lowerSecondColor = new Scalar(0, 100, 100);
                upperSecondColor = new Scalar(10, 255, 255);
            }
            Mat maskYellow = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            Mat maskSecondColor = new Mat();
            Core.inRange(hsv, lowerSecondColor, upperSecondColor, maskSecondColor);
            Mat combinedMask = new Mat();
            Core.bitwise_or(maskYellow, maskSecondColor, combinedMask);

            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double bestDistance = Double.MAX_VALUE;
            Point bestPoint = null;
            String bestColor = "None";

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                double distance = Math.sqrt(Math.pow(center.x - BOTTOM_CENTER_X, 2) + Math.pow(center.y - BOTTOM_CENTER_Y, 2));

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestPoint = center;


                    if (Core.countNonZero(maskYellow) > 0) {
                        bestColor = "Yellow";
                    } else if (Core.countNonZero(maskSecondColor) > 0) {
                        bestColor = (Globals.ALLIANCE.equals(Color.RED)) ? "Red" : "Blue";
                    }
                }
            }

            bestTarget = webcamKalman.update(bestPoint);
            selectedSampleColor = bestColor;
            return input;
        }

        public Point getBestTarget() {
            return bestTarget;
        }
    }
}

class KalmanFilter {
    private double x = 0, y = 0;
    private double p = 1, q = 0.1, r = 0.1;

    public Point update(Point measurement) {
        if (measurement == null) return new Point(x, y);
        double k = p / (p + r);
        x = x + k * (measurement.x - x);
        y = y + k * (measurement.y - y);
        p = (1 - k) * p + q;
        return new Point(x, y);
    }
}
