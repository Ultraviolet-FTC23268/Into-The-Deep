package org.firstinspires.ftc.teamcode.Common.Vision;

import static java.lang.Math.abs;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.Nullable;
import org.opencv.android.Utils;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.opencv.core.Rect;


@Config
public class DetectionPipeline implements CameraStreamSource, VisionProcessor
{
    // AtomicReference to store the last frame as a Bitmap
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /*
     * Our working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    public double AREA = 0;
    public static int MIN_AREA = 100000;
    public static int MAX_AREA = 600000;

    public static Scalar LOWER_BLUE = new Scalar(100, 150, 50);  // HSV range for blue
    public static Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    public static Scalar LOWER_RED_1 = new Scalar(0, 150, 50);   // HSV range for lower red
    public static Scalar UPPER_RED_1 = new Scalar(10, 255, 255);

    public static Scalar LOWER_RED_2 = new Scalar(170, 150, 50); // HSV range for upper red
    public static Scalar UPPER_RED_2 = new Scalar(180, 255, 255);

    public static Scalar LOWER_YELLOW = new Scalar(20, 150, 50); // HSV range for yellow
    public static Scalar UPPER_YELLOW = new Scalar(30, 255, 255);
    public static int CutoffLine = 960;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    /*
     * Colors
     */
    public static final Scalar RED = new Scalar(0.0, 100.0, 100.0);
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar YELLOW = new Scalar(20.0, 100.0, 100.0);

    public static final int CONTOUR_LINE_THICKNESS = 2;

    public static class AnalyzedSample
    {
        double angle;
        String color;
        org.firstinspires.ftc.teamcode.Common.Drive.geometry.Point translate;

        public double getAngle() {
            return angle;
        }

        public org.firstinspires.ftc.teamcode.Common.Drive.geometry.Point getTranslate() {
            return translate;
        }
    }

    ArrayList<AnalyzedSample> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedSample> clientStoneList = new ArrayList<>();

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();

    // Keep track of what stage the viewport is showing
    public static int stageNum = 0;

    public SampleType sampleType = SampleType.Blue;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize the last frame with the correct size
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    public DetectionPipeline()
    {
        // Initialize camera parameters
        // Replace these values with your actual camera calibration parameters

        // Focal lengths (fx, fy) and principal point (cx, cy)
        double fx = 1414.79; // Replace with your camera's focal length in pixels
        double fy = 1414.79;
        double cx = 940.525; // Replace with your camera's principal point x-coordinate (usually image width / 2)
        double cy = 555.64; // Replace with your camera's principal point y-coordinate (usually image height / 2)

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        // Distortion coefficients (k1, k2, p1, p2, k3)
        // If you have calibrated your camera and have these values, use them
        // Otherwise, you can assume zero distortion for simplicity
        distCoeffs = new MatOfDouble( 0.0847076, -0.31114, 0.00199819, 0.0043678, 0.385143);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        // We'll be updating this with new data below
        internalStoneList.clear();

        // Draw a line on the bottom 1/8 of the screen
        //Imgproc.line(input, new Point(0, CutoffLine), new Point(input.width(), CutoffLine), new Scalar(255, 0, 0), 2);


        /*
         * Run the image processing
         */
        findContours(input);

        clientStoneList = new ArrayList<>(internalStoneList);

        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum])
        {
            case YCrCb:
            {
                Bitmap bitmap;
                if (sampleType == SampleType.Yellow) {
                    bitmap = Bitmap.createBitmap(yellowThresholdMat.width(), yellowThresholdMat.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(yellowThresholdMat, bitmap);
                } else if (sampleType == SampleType.Blue) {
                    bitmap = Bitmap.createBitmap(blueThresholdMat.width(), blueThresholdMat.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(blueThresholdMat, bitmap);
                } else {
                    bitmap = Bitmap.createBitmap(redThresholdMat.width(), redThresholdMat.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(redThresholdMat, bitmap);
                }

                // Update the last frame
                lastFrame.set(bitmap);
                return ycrcbMat;
            }

            case FINAL:
            {
                Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(input, bitmap);

                // Update the last frame
                lastFrame.set(bitmap);
                return input;
            }

            case MASKS:
            {
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);

                Bitmap bitmap = Bitmap.createBitmap(masks.width(), masks.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(masks, bitmap);

                // Update the last frame
                lastFrame.set(bitmap);
                return masks;
            }

            case MASKS_NR:
            {
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);

                Bitmap bitmap = Bitmap.createBitmap(masksNR.width(), masksNR.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(masksNR, bitmap);

                // Update the last frame
                lastFrame.set(bitmap);
                return masksNR;
            }

            case CONTOURS:
            {
                Bitmap bitmap = Bitmap.createBitmap(contoursOnPlainImageMat.width(), contoursOnPlainImageMat.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(contoursOnPlainImageMat, bitmap);

                // Update the last frame
                lastFrame.set(bitmap);

                return contoursOnPlainImageMat;
            }
        }

        return input;
    }

    public ArrayList<AnalyzedSample> getDetectedStones()
    {
        return clientStoneList;
    }

    @Nullable
    public AnalyzedSample chooseClosestValidSample() {
        if (clientStoneList.isEmpty()) {
            return null;
        }

        AnalyzedSample closestStone = clientStoneList.get(0);
        double closestDist = closestStone.translate.radius();
        for (AnalyzedSample stone : clientStoneList) {
            if (!(abs(stone.translate.x) > 290) && !(abs(stone.translate.y) > 200)) {
                if (stone.translate.radius() < closestDist) {
                    closestStone = stone;
                }
            }
        }
        return closestStone;
    }

    void findContours(Mat input) {
        // Convert the input image to HSV color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2HSV);

        if (sampleType == SampleType.Red) {
            // Handle red, as it appears at both ends of the hue spectrum
            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(ycrcbMat, LOWER_RED_1, UPPER_RED_1, redMask1);
            Core.inRange(ycrcbMat, LOWER_RED_2, UPPER_RED_2, redMask2);
            Core.addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redThresholdMat);
        } else if (sampleType == SampleType.Blue) {
            Core.inRange(ycrcbMat, LOWER_BLUE, UPPER_BLUE, blueThresholdMat);
        } else if (sampleType == SampleType.Yellow) {
            Core.inRange(ycrcbMat, LOWER_YELLOW, UPPER_YELLOW, yellowThresholdMat);
        }

        if (sampleType == SampleType.Blue) {
            morphMask(blueThresholdMat, morphedBlueThreshold);
        } else if (sampleType == SampleType.Red) {
            morphMask(redThresholdMat, morphedRedThreshold);
        } else if (sampleType == SampleType.Yellow) {
            morphMask(yellowThresholdMat, morphedYellowThreshold);
        }

        // Find contours in the masks
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        Imgproc.findContours(
                sampleType == SampleType.Red ? morphedRedThreshold
                        : (sampleType == SampleType.Blue ? morphedBlueThreshold : morphedYellowThreshold),
                contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE
        );

        // Analyze detected contours
        for (MatOfPoint contour : contoursList) {
            // Check if the contour is above the line
            //if (isContourAboveLine(contour)) {
                analyzeContour(contour, input, sampleType.name());
            //}
        }
    }

    boolean isContourAboveLine(MatOfPoint contour) {
        // Check the y-coordinates of the contour's bounding box
        Rect boundingBox = Imgproc.boundingRect(contour);
        return boundingBox.y + boundingBox.height < CutoffLine;
    }


    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        AREA = rotatedRectFitToContour.size.area();
        if (rotatedRectFitToContour.size.area() < MIN_AREA || rotatedRectFitToContour.size.area() > MAX_AREA) {
            return;
        }
        drawRotatedRect(rotatedRectFitToContour, input, color);

        // The angle OpenCV gives us can be ambiguous, so look at the shape of
        // the rectangle to fix that.
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
        {
            rotRectAngle += 90;
        }

        // Compute the angle and store it
        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle)) + " deg", input, color);

        // Prepare object points and image points for solvePnP
        // Assuming the object is a rectangle with known dimensions
        double objectWidth = 10.0;  // Replace with your object's width in real-world units (e.g., centimeters)
        double objectHeight = 5.0;  // Replace with your object's height in real-world units

        // Define the 3D coordinates of the object corners in the object coordinate space
        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        // Get the 2D image points from the detected rectangle corners
        Point[] rectPoints = new Point[4];
        rotatedRectFitToContour.points(rectPoints);

        // Order the image points in the same order as object points
        Point[] orderedRectPoints = orderPoints(rectPoints);

        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

        // Solve PnP
        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(
                objectPoints, // Object points in 3D
                imagePoints,  // Corresponding image points
                cameraMatrix,
                distCoeffs,
                rvec,
                tvec
        );

        if (success)
        {
            // Draw the coordinate axes on the image
            drawAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            double xcord = 1080 - rotatedRectFitToContour.center.y;
            double ycord = -rotatedRectFitToContour.center.x + 1920.0 / 2;
            double convertedX = xcord/7.685; //change from pixels to mm
            double convertedY = ycord/7.685;

            // Store the pose information
            AnalyzedSample analyzedSample = new AnalyzedSample();
            analyzedSample.angle = rotRectAngle;
            analyzedSample.color = color;
            analyzedSample.translate = new org.firstinspires.ftc.teamcode.Common.Drive.geometry.Point(convertedX, convertedY);

            internalStoneList.add(analyzedSample);
        }
    }

    void drawAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs)
    {
        // Length of the axis lines
        double axisLength = 5.0;

        // Define the points in 3D space for the axes
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axisLength, 0, 0),
                new Point3(0, axisLength, 0),
                new Point3(0, 0, -axisLength) // Z axis pointing away from the camera
        );

        // Project the 3D points to 2D image points
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();

        // Draw the axis lines
        Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2); // X axis in red
        Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2); // Y axis in green
        Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2); // Z axis in blue
    }

    static Point[] orderPoints(Point[] pts)
    {
        // Orders the array of 4 points in the order: top-left, top-right, bottom-right, bottom-left
        Point[] orderedPts = new Point[4];

        // Sum and difference of x and y coordinates
        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++)
        {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        // Top-left point has the smallest sum
        int tlIndex = indexOfMin(sum);
        orderedPts[0] = pts[tlIndex];

        // Bottom-right point has the largest sum
        int brIndex = indexOfMax(sum);
        orderedPts[2] = pts[brIndex];

        // Top-right point has the smallest difference
        int trIndex = indexOfMin(diff);
        orderedPts[1] = pts[trIndex];

        // Bottom-left point has the largest difference
        int blIndex = indexOfMax(diff);
        orderedPts[3] = pts[blIndex];

        return orderedPts;
    }

    static int indexOfMin(double[] array)
    {
        int index = 0;
        double min = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] < min)
            {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array)
    {
        int index = 0;
        double max = array[0];

        for (int i = 1; i < array.length; i++)
        {
            if (array[i] > max)
            {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color)
    {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color)
    {
        switch (color)
        {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> {
            // Pass the last frame (Bitmap) to the consumer
            bitmapConsumer.accept(lastFrame.get());
        });
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // Optional: If you want to draw additional information on the canvas
    }
}