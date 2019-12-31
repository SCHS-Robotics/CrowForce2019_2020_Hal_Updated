package org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
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

public class opencvSkystoneDetector_v2 extends SubSystem {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1, valUp = -1, valDown = -1, bitset = 0;
    public static int skystoneInd = 0;
    private static double rectHeight = 1.5 / 8.0, rectWidth = 0.6 / 8.0;

    private static double offsetX = 2.0 / 8.0; //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static double offsetY = 0.0; //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static double[] midPos = {4.0 / 8.0 + offsetX, 4.0 / 8.0 + offsetY};//0 = col, 1 = row
    private static double[] upPos = {4.0 / 8.0 + offsetX, 2.5 / 8.0 + offsetY};
    private static double[] downPos = {4.0 / 8.0 + offsetX, 5.5 / 8.0 + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private int rows = 640, cols = 480;
    OpenCvCamera phoneCam;

    /**
     * @param robot - The robot the subsystem is contained within.
     */
    public opencvSkystoneDetector_v2(Robot robot) {
        super(robot);
    }

    @Override
    public void init()  {

    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {

    }

    @Override
    public void handle()  {

    }

    @Override
    public void stop()  {

    }
}
