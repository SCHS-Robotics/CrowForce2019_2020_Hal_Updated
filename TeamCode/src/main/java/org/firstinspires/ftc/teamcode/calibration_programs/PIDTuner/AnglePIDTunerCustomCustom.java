package org.firstinspires.ftc.teamcode.calibration_programs.PIDTuner;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.Robots.MainRobot;

public class AnglePIDTunerCustomCustom extends SubSystem {
    BNO055IMU imu;
    public AnglePIDTunerCustomCustom(Robot r) {
        super(r);
        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");
    }

    double time1 = System.currentTimeMillis();
    double time2 = System.currentTimeMillis();
    double desAng = Math.PI/4;
    //double currentPos = imu.getPosition();
    //double errFunc = imu.getPosition() - desAng;
    //double slope = (imu.getPosition() - desAng/(time2 - time1));


    public void setTime1(){
        time1 = System.currentTimeMillis();
    }

    @Override
    public void init() throws InterruptedException {

    }
    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {
   // double time1 = robot.;
    }

    @Override
    public void handle() throws InterruptedException {


    }

    @Override
    public void stop() throws InterruptedException {

    }
}
