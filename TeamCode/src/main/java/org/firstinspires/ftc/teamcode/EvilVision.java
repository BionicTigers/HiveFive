package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//EvilVision be like: Exists
@Config
public class EvilVision extends OpenCvPipeline {
    private static int mode=1; //
    private static double area; //represents the area of the contours around the rings
    private Mat hslThresholdOutput = new Mat();
    public static double minHue = 150;
    public static double maxHue = 180;
    public static double minSat = 50;
    public static double maxSat = 255;
    public static double minLum = 45;
    public static double maxLum = 235;
    public double x = 5;
    public double y = 0;
    public Rect rect;
    public static List<MatOfPoint> Shippingelement = new ArrayList<>();
    int i = -1;


   // double[] hslThresholdHue = {3, 23};
//    double[] hslThresholdSaturation = {50, 255};
//    double[] hslThresholdLuminance = {45, 235};

    private Mat one;
    public static boolean showHSL;

    public EvilVision(OpenCvCamera cam) {

    }

    public EvilVision() {

    }

    /**
     * @param input - input matrix
     * @param hue   - values for hue
     * @param sat   - values for saturation
     * @param lum   - values for luminance
     * @param out   - output matrix
     *              takes an input matrix and applies a filter into the output matrix
     *              filter uses values of hue, saturation, and luminance
     */
    private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HLS);
        Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                new Scalar(hue[1], lum[1], sat[1]), out);
    }
    public Mat getHslThresholdOutput() {
        return hslThresholdOutput;
    }

    /**
     * I don't use this method
     */
    public double ratioJudge(double height, double width, double ratioWant) {
        double ratio = height / width;
        return Math.abs(ratioWant - ratio);
    }

    /**
     * @param source0 this is the camera input
     *                This method find and draws the contours on the rings
     * @return
     */
@Override

    public Mat processFrame(Mat source0) {
        Mat hiarchy = new Mat();
//
//    double[] hslThresholdHue = {3, 23};
//    double[] hslThresholdSaturation = {50, 255};
//    double[] hslThresholdLuminance = {45, 235};

    double[] hslThresholdHue = {minHue, maxHue};
    double[] hslThresholdSaturation = {minSat, maxSat};
    double[] hslThresholdLuminance = {minLum, maxLum};
        //takes values for hue, saturation, and luminance and apply's them to what the camera sees
        hslThreshold(source0, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

   Imgproc.GaussianBlur(source0, source0, new Size(9,9), 0);


        //adds a blur to what the camera sees


        //finds contours from what the camera sees
        Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);



   for(MatOfPoint con :contoursBlack){
       if(Imgproc.contourArea(con) >= 500 ){
           Shippingelement.add(con);
           i++;
       }
   }

   //uses the matrix from the last loop and draws the contours for the rings over what the camera sees
    if(showHSL){
        source0 = hslThresholdOutput;

    }




   //gets the area of the contours around the rings
   if(Shippingelement.size() > 0){
       rect = Imgproc.boundingRect(Shippingelement.get(i));
       x = rect.x + rect.width/2.0;
       y = rect.y + rect.height / 2.0;
       Imgproc.drawContours(source0, Shippingelement, -1, new Scalar(250,0,250),3);
       Imgproc.rectangle(source0, rect, new Scalar(0,255,0));
       area = Imgproc.contourArea(Shippingelement.get(i));
   }
   else {
       area = 0;
   }

   Elementlocation();
//
//         double[] hslThresholdHue2 = {24,152};
//         double[] hslThresholdSaturation2 = {0,49};
//         double[] hslThresholdLuminance2 = {151, 255};
//         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
//         hslThreshold(source0, hslThresholdHue2, hslThresholdSaturation2, hslThresholdLuminance2, hslThresholdOutput);
//
//         //adds a blur to what the camera sees
//         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
//
//         //finds contours from what the camera sees
//         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//         List<MatOfPoint> Plane = new ArrayList<>();
//         double[] hslThresholdHueDuck = {19, 180};
//         double[] hslThresholdSaturationDuck = {124,255};
//         double[] hslThresholdLuminanceDuck = {127, 255};
//         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
//         hslThreshold(source0, hslThresholdHueDuck, hslThresholdSaturationDuck, hslThresholdLuminanceDuck, hslThresholdOutput);
//         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
//         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//         List<MatOfPoint> Duck = new ArrayList<>();
//
//         List<MatOfPoint> contoursBlack = new ArrayList<>();

//         double[] hslThresholdHueRsquare = {, };
//         double[] hslThresholdSaturationRsquare = {,};
//         double[] hslThresholdLuminanceRsquare = {,};
//         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
//         //Object is for the team shipping element
//         hslThreshold(source0, hslThresholdHueRsquare, hslThresholdSaturationRsquare, hslThresholdLuminanceRsquare, hslThresholdOutput);
//         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
//         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//         List<MatOfPoint> Redsquare = new ArrayList<>();


        return source0;

    }
    public double getArea(){
        return area;
    }
    public int getRectx() {return rect.x;}

    //based of the area of the contours, this method finds the number of rings the camera is seeing (0, 1, 4)
    public void Elementlocation(){
        if(area <= 500)
            mode = 1;
        else if(area > 6500)
            mode = 3;
        else if(area <= 6500)
            mode =2;
    }
    public int getMode(){
        return mode;
    }

}