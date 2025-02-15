package org.firstinspires.ftc.teamcode.Common.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import java.util.List;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;


public class SampleDistancePipeline extends OpenCvPipeline {

    public String selectedSampleColor = "Red";

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);
        Scalar lowerBlue = new Scalar(100, 100, 100);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        Mat mask = new Mat();
        if (selectedSampleColor.equals("Red")) {
            Core.inRange(hsv, lowerRed, upperRed, mask);
        } else if (selectedSampleColor.equals("Blue")) {
            Core.inRange(hsv, lowerBlue, upperBlue, mask);
        } else if (selectedSampleColor.equals("Yellow")) {
            Core.inRange(hsv, lowerYellow, upperYellow, mask);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect closestBoundingBox = new Rect();
        double minDistance = Double.MAX_VALUE;
        double frameCenterX = input.cols() / 2;
        double frameCenterY = input.rows() / 2;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            MatOfPoint approxCurveMat = new MatOfPoint(approxCurve.toArray());

            if (approxCurve.total() == 4) {
                Rect boundingBox = Imgproc.boundingRect(approxCurve);

                double distance = Math.sqrt(Math.pow(boundingBox.x + boundingBox.width / 2 - frameCenterX, 2) +
                        Math.pow(boundingBox.y + boundingBox.height / 2 - frameCenterY, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestBoundingBox = boundingBox;
                }

                Imgproc.polylines(input, List.of(approxCurveMat), true, new Scalar(255, 0, 0), 2);
            }
        }

        return input;
    }
}
