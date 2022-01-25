//package org.firstinspires.ftc.teamcode;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//import java.util.ArrayList;
//import java.util.List;
//
// public class Vuforia extends OpenCvPipeline {
//
//     private Mat hslThresholdOutput = new Mat();
//
//     private Mat one;
//     private static double area; //represents the area of the contours around the rings
//     private static int mode=1; //
//
//     public Vuforia(OpenCvCamera cam) {
//
//     }
//
//     public Vuforia() {
//
//     }
//
//     /**
//      * @param input - input matrix
//      * @param hue   - values for hue
//      * @param sat   - values for saturation
//      * @param lum   - values for luminance
//      * @param out   - output matrix
//      *              takes an input matrix and applies a filter into the output matrix
//      *              filter uses values of hue, saturation, and luminance
//      */
//     private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
//         Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HLS);
//         Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
//                 new Scalar(hue[1], lum[1], sat[1]), out);
//     }
//     public Mat getHslThresholdOutput() {
//         return hslThresholdOutput;
//     }
//
//     /**
//      * I don't use this method
//      */
//     public double ratioJudge(double height, double width, double ratioWant) {
//         double ratio = height / width;
//         return Math.abs(ratioWant - ratio);
//     }
//
//     /**
//      * @param source0 this is the camera input
//      *                This method find and draws the contours on the rings
//      * @return
//      */
//
//     public Mat processFrame(Mat source0) {
//         Mat hiarchy = new Mat();
//         private Mat one;
//
//         private static int mode=1; //represents how many rings the camera sees
//         private static double area; //represents the area of the contours around the rings
//
//         double[] hslThresholdHue = {5, 67};
//         double[] hslThresholdSaturation = {0, 153};
//         double[] hslThresholdLuminance = {105, 255};
//         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
//         hslThreshold(source0, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);
//         List<MatOfPoint> contoursBlack = new ArrayList<>();
//
//         //adds a blur to what the camera sees
//         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
//
//         //finds contours from what the camera sees
//         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//         List<MatOfPoint> Shipppingelement = new ArrayList<>();
////
////         double[] hslThresholdHue2 = {24,152};
////         double[] hslThresholdSaturation2 = {0,49};
////         double[] hslThresholdLuminance2 = {151, 255};
////         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
////         hslThreshold(source0, hslThresholdHue2, hslThresholdSaturation2, hslThresholdLuminance2, hslThresholdOutput);
////
////         //adds a blur to what the camera sees
////         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
////
////         //finds contours from what the camera sees
////         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
////
////         List<MatOfPoint> Plane = new ArrayList<>();
////         double[] hslThresholdHueDuck = {19, 180};
////         double[] hslThresholdSaturationDuck = {124,255};
////         double[] hslThresholdLuminanceDuck = {127, 255};
////         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
////         hslThreshold(source0, hslThresholdHueDuck, hslThresholdSaturationDuck, hslThresholdLuminanceDuck, hslThresholdOutput);
////         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
////         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
////
////         List<MatOfPoint> Duck = new ArrayList<>();
////
////         List<MatOfPoint> contoursBlack = new ArrayList<>();
//
////         double[] hslThresholdHueRsquare = {, };
////         double[] hslThresholdSaturationRsquare = {,};
////         double[] hslThresholdLuminanceRsquare = {,};
////         //takes values for hue, saturation, and luminance and apply's them to what the camera sees
////         //Object is for the team shipping element
////         hslThreshold(source0, hslThresholdHueRsquare, hslThresholdSaturationRsquare, hslThresholdLuminanceRsquare, hslThresholdOutput);
////         Imgproc.GaussianBlur(source0, source0, new Size(9, 9), 0);
////         Imgproc.findContours(hslThresholdOutput, contoursBlack, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
////
////         List<MatOfPoint> Redsquare = new ArrayList<>();
//
//
//         return source0;
//
//     }
//     public double getArea(){
//         return area;
//     }
//
//     //based of the area of the contours, this method finds the number of rings the camera is seeing (0, 1, 4)
//     public void getRingNumber(){
//         if(area <= 500 )
//             mode = 1;
//         else if(area > 1100)
//             mode = 3;
//         else
//             mode =2;
//     }
//
//
//     public int getMode(){
//         return mode;
//     }
//
// }