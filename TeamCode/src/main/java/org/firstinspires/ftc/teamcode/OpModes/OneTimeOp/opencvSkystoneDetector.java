package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp

/**
 * Garrett this is important,
 * but our future up position will be right when rotated.
 * our middle position will stay middle
 * our future down position will be left when rotated
 **/
//@Autonomous(name = "opencvSkystoneDetector", group="Sky autonomous")
public class opencvSkystoneDetector extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1, valUp = -1, valDown = -1, bitset = 0;
    public static int skystoneInd = 0;
    /*    private static int valLeft = -1;
        private static int valRight = -1;*/
    private static double rectHeight = 1.5 / 8.0, rectWidth = 0.6 / 8.0;

    private static double offsetX = 2.0 / 8.0; //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static double offsetY = 0.0; //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static double[] midPos = {4.0 / 8.0 + offsetX, 4.0 / 8.0 + offsetY};//0 = col, 1 = row
    private static double[] upPos = {4.0 / 8.0 + offsetX, 2.5 / 8.0 + offsetY};
    private static double[] downPos = {4.0 / 8.0 + offsetX, 5.5 / 8.0 + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private int rows = 640, cols = 480;
    boolean worked = false;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //OpenCvCameraFactory.getInstance().createInternalCamera();
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);  // display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("valUp:", valUp);
            telemetry.addData("valMid:", valMid);
            telemetry.addData("valDown:", valDown);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            bitset = 0;
            if (valDown == 0) {
                //telemetry.addData("Every 1st block is a skystone.", valDown);
                //skystoneInd = 1;
                ++bitset;
            }
            else if (valMid == 0) {
                //telemetry.addData("Every 2nd block is a skystone.", valMid);
                //skystoneInd = 2;
                bitset += 2;
            }
            else if (valUp == 0) {
                //telemetry.addData("Every 3rd block is a skystone.", valUp);
                //skystoneInd = 3;
                bitset += 4;
            }
            int fndCnt = 0;
            for (int i=0; i<3; ++i) {
                if ((bitset & (1 << i)) == (1 << i)) {
                    ++fndCnt;
                    skystoneInd = i + 1;
                }
            }
            if (fndCnt != 1) {
                skystoneInd = -1;
            }
            telemetry.addData("skystone index:", skystoneInd);

            telemetry.update();
            sleep(100);
            // call movement functions
            // strafe(0.4, 200);
            // moveDistance(0.4, 700);

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows() * midPos[1]), (int)(input.cols() * midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixDown = thresholdMat.get((int)(input.rows() * downPos[1]), (int)(input.cols() * downPos[0]));//gets value at circle
            valDown = (int)pixDown[0];

            double[] pixUp = thresholdMat.get((int)(input.rows() * upPos[1]), (int)(input.cols() * upPos[0]));//gets value at circle
            valUp = (int)pixUp[0];

            //create three points
            Point pointMid = new Point((int)(input.cols() * midPos[0]), (int)(input.rows() * midPos[1]));
            Point pointDown = new Point((int)(input.cols() * downPos[0]), (int)(input.rows() * downPos[1]));
            Point pointUp = new Point((int)(input.cols() * upPos[0]), (int)(input.rows() * upPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointDown,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointUp,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle( // 1-3
                    all,
                    new Point(
                            input.cols() * (downPos[0] - rectWidth / 2),
                            input.rows() * (downPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (downPos[0] + rectWidth / 2),
                            input.rows() * (downPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle( // 3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle( // 5-7
                    all,
                    new Point(
                            input.cols() * (upPos[0] - rectWidth / 2),
                            input.rows() * (upPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (upPos[0] + rectWidth / 2),
                            input.rows() * (upPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

    }
}