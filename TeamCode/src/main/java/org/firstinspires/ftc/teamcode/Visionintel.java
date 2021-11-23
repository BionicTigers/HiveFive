///*
//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//    public class Visionintel extends OpenCvPipeline{
//
//        private static int mode=1; //represents how many rings the camera sees
//        private static double area; //represents the area of the contours around the rings
//        private Mat hslThresholdOutput = new Mat();
//
//        private Mat one;
//
//        public Visionintel(OpenCvCamera cam){
//
//        }
//
//        public Visionintel() {
//
//        }
//
//*
//         *
//         * @param input - input matrix
//         * @param hue - values for hue
//         * @param sat - values for saturation
//         * @param lum - values for luminance
//         * @param out - output matrix
//         * takes an input matrix and applies a filter into the output matrix
//         * filter uses values of hue, saturation, and luminance
//
//
//        private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
//            Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HLS);
//            Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
//                    new Scalar(hue[1], lum[1], sat[1]), out);
//        }
//
//        public Mat getHslThresholdOutput() {
//            return hslThresholdOutput;
//        }
//
//*
//         * I don't use this method
//
//
//        public double ratioJudge(double height, double width, double ratioWant) {
//            double ratio = height/width;
//            return Math.abs(ratioWant-ratio);
//        }
//
//*
//         *
//         * @param source0 this is the camera input
//         * This method find and draws the contours on the rings
//         * @return
//
//
//        @Override
//
//        public Mat processFrame(Mat source0) {
//            Mat hiarchy  = new Mat();
//
//            double[] hslThresholdHue = {0, 54};
//            double[] hslThresholdSaturation = {0, 255};
//            double[] hslThresholdLuminance = {6, 255};
//            //takes values for hue, saturation, and luminance and apply's them to what the camera sees
//            hslThreshold(source0, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);
//            List<MatOfPoint> contoursBlack= new ArrayList<>();
//
//            //adds a blur to what the camera sees
//            Imgproc.GaussianBlur(source0, source0, new Size(9,9), 0);
//
//            //finds contours from what the camera sees
//            Imgproc.findContours(hslThresholdOutput,contoursBlack,hiarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
//
//            //seperates the contours from the rings into its own matrix
//            List<MatOfPoint> findBlock = new ArrayList<>();
//            for(MatOfPoint con :contoursBlack){
//                if(Imgproc.contourArea(con) >= 500){
//                    findBlock.add(con);
//                }
//            }
//
//            //uses the matrix from the last loop and draws the contours for the rings over what the camera sees
//            Imgproc.drawContours(source0, findBlock, -1, new Scalar(250,0,250),1);
//
//            //gets the area of the contours around the rings
//            if(findBlock.size() > 0){
//                area = Imgproc.contourArea(findBlock.get(0));
//            }
//            else {
//                area = 0;
//            }
//
//            getRingNumber();
//            //returns what the camera sees
//            return source0;
//        }
//public Mat processFrame(Mat source0) {
//        Mat hiarchy = new Mat();
//        double[] hslThresholdHue = {0, 9};
//        double[] hslThresholdSaturation = {90, 216};
//        double[] hslThresholdLuminance = {95, 235};
//        hslThreshold(source0, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);
//
//
//        List<MatOfPoint> contoursBlack= new ArrayList<>();
//
//        Imgproc.findContours(hslThresholdOutput,contoursBlack,hiarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
//
//
//    }
//
//
//
//        //gets the area field
//        public double getArea(){
//            return area;
//        }
//
//        //based of the area of the contours, this method finds the number of rings the camera is seeing (0, 1, 4)
//        public void getRingNumber(){
//            if(area <= 500 )
//                mode = 1;
//            else if(area > 1100)
//                mode = 3;
//            else
//                mode =2;
//        }
//
//        //mode is what auto we're going to need to run
//        public int getMode(){
//            return mode;
//        }
//    }
//
//}
//*/
