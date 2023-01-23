package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;

import org.openftc.easyopencv.OpenCvPipeline;


public class ConeDetectionPipeline extends OpenCvPipeline {
    // CYAN = 1
    // MAGENTA = 2
    // YELLOW = 3

    public static final double[] RED = new double[]{255, 0, 0};
    public static final double[] GREEN = new double[]{0, 255, 0};
    public static final double[] BLUE = new double[]{0, 0, 255};
    public static double[] rgba = new double[3];
    public static int latestResult = 0;

    public static double euclidianDistance(double[] color1, double[] color2) {
        return Math.sqrt(
            Math.pow(color1[0] - color2[0], 2) +
            Math.pow(color1[1] - color2[1], 2) +
            Math.pow(color1[2] - color2[2], 2)
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        rgba = input.get(input.cols()/2, input.rows()/2);

        double red_dist = euclidianDistance(rgba, RED);
        double green_dist = euclidianDistance(rgba, GREEN);
        double blue_dist = euclidianDistance(rgba, BLUE);

        if ((red_dist < green_dist) && (red_dist < blue_dist)) {
            latestResult = 1;
        } else if ((blue_dist < red_dist) && (blue_dist < green_dist)) {
            latestResult = 2;
        } else if ((green_dist < red_dist) && (green_dist < blue_dist)) {
            latestResult = 3;
        }
        
        return input;
    }

    public static int getLatestResult() {
        return latestResult;
    }
    public static double[] getRGBA() {return rgba;}
}