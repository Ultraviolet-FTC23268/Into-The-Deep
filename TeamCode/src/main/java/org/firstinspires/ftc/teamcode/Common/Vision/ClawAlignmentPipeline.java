package org.firstinspires.ftc.teamcode.Common.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import androidx.annotation.NonNull;

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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ClawAlignmentPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private double blockAngle = 0.0;
    private double smoothedAngle = 0.0;
    private double estimatedAngle = 0.0;
    private double allianceColor = 0;
    private final List<Double> angleHistory = new ArrayList<>();
    private final int historySize = 5;
    private final double hysteresisThreshold = 5.0;
    private double processNoise = 1.0;
    private double measurementNoise = 5.0;
    private double errorCovariance = 1.0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        allianceColor = Globals.ALLIANCE == Color.BLUE ? 0 : 1;
    }

    @Override
    public Mat processFrame(Mat input, long timestamp) {
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
        if (allianceColor == 1) {
            Core.bitwise_or(redMask, yellowMask, combinedMask);
        } else {
            Core.bitwise_or(blueMask, yellowMask, combinedMask);
        }

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint closestContour = null;
        double minDistance = Double.MAX_VALUE;
        int frameWidth = input.width();
        int frameHeight = input.height();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 1000) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                int blockCenterX = boundingRect.x + boundingRect.width / 2;
                int blockCenterY = boundingRect.y + boundingRect.height / 2;
                double distance = Math.sqrt(Math.pow(blockCenterX - frameWidth / 2, 2) + Math.pow(blockCenterY - frameHeight / 2, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestContour = contour;
                }
            }
        }

        if (closestContour != null) {
            MatOfPoint2f contour2f = new MatOfPoint2f(closestContour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f simplifiedContour = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, simplifiedContour, epsilon, true);

            Imgproc.drawContours(input, List.of(new MatOfPoint(simplifiedContour.toArray())), -1, new Scalar(0, 255, 0), 2);

            Point[] longestEdge = findLongestEdge(simplifiedContour.toArray(), frameWidth, frameHeight, 10);

            if (longestEdge[0] != null && longestEdge[1] != null) {
                Point p1 = longestEdge[0];
                Point p2 = longestEdge[1];
                Imgproc.line(input, p1, p2, new Scalar(255, 0, 0), 2);
                blockAngle = calculateEdgeAngle(p1, p2, frameHeight);
            }
        }

        angleHistory.add(blockAngle);
        if (angleHistory.size() > historySize) {
            angleHistory.remove(0);
        }

        List<Double> sortedAngles = new ArrayList<>(angleHistory);
        Collections.sort(sortedAngles);
        double medianAngle = sortedAngles.get(sortedAngles.size() / 2);

        smoothedAngle = 0.9 * smoothedAngle + 0.1 * medianAngle;

        if (Math.abs(blockAngle - smoothedAngle) > 30) {
            blockAngle = smoothedAngle;
        }
        if (Math.abs(smoothedAngle - estimatedAngle) > hysteresisThreshold) {
            estimatedAngle = smoothedAngle;
        }

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
        Point[] longestEdge = {null, null};
        double maxDistance = 0.0;

        for (int i = 0; i < contour.length; i++) {
            Point p1 = contour[i];
            Point p2 = contour[(i + 1) % contour.length];
            double distance = Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));

            if (distance > maxDistance && p1.x > borderThreshold && p2.x > borderThreshold && p1.x < frameWidth - borderThreshold && p2.x < frameWidth - borderThreshold && p1.y > borderThreshold && p2.y > borderThreshold && p1.y < frameHeight - borderThreshold && p2.y < frameHeight - borderThreshold) {
                maxDistance = distance;
                longestEdge[0] = p1;
                longestEdge[1] = p2;
            }
        }
        return longestEdge;
    }

    private double calculateEdgeAngle(Point p1, Point p2, int frameHeight) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Bitmap frame = lastFrame.get();
        if (frame != null) {
            canvas.drawBitmap(frame, null, new android.graphics.Rect(0, 0, onscreenWidth, onscreenHeight), null);
        }

        android.graphics.Paint textPaint = new android.graphics.Paint();
        textPaint.setColor(android.graphics.Color.WHITE);
        textPaint.setTextSize(30 * scaleCanvasDensity);
        canvas.drawText(String.format("Estimated Angle: %.2f", estimatedAngle), 10, 50, textPaint);
        canvas.drawText(String.format("Smoothed Angle: %.2f", smoothedAngle), 10, 90, textPaint);
    }

    @Override
    public void getFrameBitmap(@NonNull Continuation<? extends Consumer<Bitmap>> continuation) {
        Bitmap frame = lastFrame.get();
        continuation.dispatch(consumer -> consumer.accept(frame));
    }

    public double getSampleAngle() {
        return estimatedAngle;
    }
}
