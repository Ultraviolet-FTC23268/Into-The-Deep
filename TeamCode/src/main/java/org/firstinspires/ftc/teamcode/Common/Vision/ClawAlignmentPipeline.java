package org.firstinspires.ftc.teamcode.Common.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Point;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class ClawAlignmentPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private double blockAngle = 0.0;
    private double avgAngle = 0;
    private double allianceColor = 0;

    public static double estimatedAngle = 0.0;
    public static double processNoise = 3;
    public static double measurementNoise = 150;
    public static double errorCovariance = 7.5;
    public static int historySize = 1;
    private final double hysteresisThreshold = 10.0;
    public static double alpha = 0.01;
    public static double minChange = 7.5;

    private List<Double> angleHistory = new ArrayList<>();
    private double smoothedAngle = 0.0;
    private double filteredAngle = 90;

    private double finalAngle = 90;

    private static final int SKIP_FRAME_INTERVAL = 3; // Process every nth frame
    private int frameCounter = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        if (Globals.ALLIANCE == Color.BLUE)
            allianceColor = 0;
        else
            allianceColor = 1;
    }

    @Override
    public Mat processFrame(Mat input, long bruh) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat blueMask = new Mat();
        Mat yellowMask = new Mat();

        Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), redMask1);
        Core.inRange(hsv, new Scalar(170, 120, 70), new Scalar(180, 255, 255), redMask2);
        Mat redMask = new Mat();
        Core.bitwise_or(redMask1, redMask2, redMask);

        Core.inRange(hsv, new Scalar(20, 100, 100), new Scalar(30, 255, 255), yellowMask);

        Core.inRange(hsv, new Scalar(100, 100, 50), new Scalar(140, 255, 255), blueMask);

        Mat combinedMask = new Mat();
        if (allianceColor == 1)
            Core.bitwise_or(redMask, yellowMask, combinedMask);
        else
            Core.bitwise_or(blueMask, yellowMask, combinedMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int frameWidth = input.width();
        int frameHeight = input.height();
        final double[] minDistance = {Double.MAX_VALUE}; // Making minDistance final
        final MatOfPoint[] closestContour = {null}; // Making closestContour final

        contours.parallelStream().forEach(contour -> {
            double area = Imgproc.contourArea(contour);
            if (area > 1500) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                int blockCenterX = boundingRect.x + boundingRect.width / 2;
                int blockCenterY = boundingRect.y + boundingRect.height / 2;
                double distance = Math.sqrt(Math.pow(blockCenterX - frameWidth / 2, 2) + Math.pow(blockCenterY - frameHeight / 2, 2));

                synchronized (minDistance) {
                    if (distance < minDistance[0]) {
                        minDistance[0] = distance;
                        closestContour[0] = contour;
                    }
                }
            }
        });

        if (closestContour[0] != null) {
            MatOfPoint2f contour2f = new MatOfPoint2f(closestContour[0].toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f simplifiedContour = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, simplifiedContour, epsilon, true);

            Imgproc.drawContours(input, List.of(new MatOfPoint(simplifiedContour.toArray())), -1, new Scalar(0, 255, 0), 2);

            Point[] longestEdge = findLongestEdge(simplifiedContour.toArray(), frameWidth, frameHeight, 10);

            if (longestEdge[0] != null && longestEdge[1] != null) {
                Point p1 = longestEdge[0];
                Point p2 = longestEdge[1];

                Imgproc.line(input, p1, p2, new Scalar(255, 0, 0), 2);

                blockAngle = calculateEdgeAngle(p1, p2, frameHeight, input);
                Imgproc.putText(input, String.format("Angle: %.2f", blockAngle), new Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 2);
            }
        }

        double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        estimatedAngle = estimatedAngle + kalmanGain * (blockAngle - estimatedAngle);
        errorCovariance = (1 - kalmanGain) * errorCovariance + processNoise;

        if (angleHistory.size() >= historySize) {
            angleHistory.remove(0);
        }
        angleHistory.add(estimatedAngle);
        avgAngle = calculateMedianAngle(angleHistory);

        smoothedAngle = applyLowPassFilter(avgAngle);

        if (Math.abs(filteredAngle - avgAngle) > minChange)
            filteredAngle = avgAngle;

        finalAngle = applyIncrementation(filteredAngle);

        hsv.release();
        redMask1.release();
        redMask2.release();
        yellowMask.release();
        blueMask.release();
        combinedMask.release();
        kernel.release();
        hierarchy.release();

        return input;
    }

    private Point[] findLongestEdge(Point[] contour, int frameWidth, int frameHeight, int borderThreshold) {
        double maxDistance = 0;
        Point[] longestEdge = new Point[]{null, null};

        for (int i = 0; i < contour.length; i++) {
            Point p1 = contour[i];
            Point p2 = contour[(i + 1) % contour.length];

            boolean p1TooClose = (p1.x < borderThreshold || p1.x > frameWidth - borderThreshold ||
                    p1.y < borderThreshold || p1.y > frameHeight - borderThreshold);

            boolean p2TooClose = (p2.x < borderThreshold || p2.x > frameWidth - borderThreshold ||
                    p2.y < borderThreshold || p2.y > frameHeight - borderThreshold);

            if (p1TooClose && p2TooClose) continue;

            double distance = Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));

            if (distance > maxDistance) {
                maxDistance = distance;
                longestEdge[0] = p1;
                longestEdge[1] = p2;
            }
        }

        return longestEdge;
    }

    private double calculateEdgeAngle(Point p1, Point p2, int frameHeight, Mat frame) {
        if (p1.y > p2.y) {
            Point temp = p1;
            p1 = p2;
            p2 = temp;
        }

        Point bottomPoint = new Point(p1.x, frameHeight);

        Imgproc.line(frame, bottomPoint, new Point(p1.x, 0), new Scalar(0, 255, 255), 2);

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double edgeAngleRadians = Math.atan2(dy, dx);
        double edgeAngleDegrees = Math.toDegrees(edgeAngleRadians);
        double relativeAngle = edgeAngleDegrees >= 0 ? edgeAngleDegrees : 180 + edgeAngleDegrees;

        return relativeAngle;
    }

    private double calculateMedianAngle(List<Double> angles) {
        List<Double> sortedAngles = new ArrayList<>(angles);
        sortedAngles.sort(Double::compareTo);
        int size = sortedAngles.size();
        if (size % 2 == 0) {
            return (sortedAngles.get(size / 2 - 1) + sortedAngles.get(size / 2)) / 2;
        } else {
            return sortedAngles.get(size / 2);
        }
    }

    private double applyLowPassFilter(double newAngle) {
        smoothedAngle = alpha * newAngle + (1 - alpha) * smoothedAngle;
        return smoothedAngle;
    }

    private double applyIncrementation(double filteredAngle) {
        finalAngle = Math.round(filteredAngle / 45.0) * 45.0;
        return finalAngle;
    }

    public double getSampleAngle() {
        return finalAngle;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
    }
}
