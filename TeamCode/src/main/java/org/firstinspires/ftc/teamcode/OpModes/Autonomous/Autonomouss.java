package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import android.util.Log;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.SCHSRobotics.HAL9001.util.calib.EncoderDistanceCalib;
import com.SCHSRobotics.HAL9001.util.exceptions.DumpsterFireException;
import com.SCHSRobotics.HAL9001.util.exceptions.InvalidMoveCommandException;
import com.SCHSRobotics.HAL9001.util.exceptions.WrongDrivetypeException;
import com.SCHSRobotics.HAL9001.util.math.EncoderToDistanceProcessor;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robots.MainRobot;

import java.util.Base64;

import static java.lang.Math.PI;
import static java.lang.Thread.sleep;


@StandAlone
@Autonomous (name = "Autonomous")
public class Autonomouss extends BaseAutonomous {
    private MainRobot robot;
    private EncoderToDistanceProcessor processor = new EncoderToDistanceProcessor(2.5, Units.INCH);

    //list of common encoder distances:
    

    //1 tile
    int oneTile = processor.getEncoderAmount(23, Units. INCH);
    //2 tiles
    int twoTile = processor.getEncoderAmount(46, Units. INCH);
    //2.5 tiles (right before of intake)
    int blockPos = processor.getEncoderAmount(7, Units. INCH);


    private void turnEncoders(double power, int encoder){
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.turn(power);
        while(Math.abs(robot.distance.fEncoders() - encoderStart) < encoder){
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            if(encoder - temp < 1440) {
                robot.mDrive.turn(((encoder - temp)/1440)*power);
            }
        }
        robot.mDrive.stopAllMotors();
    }
    private void strafeEncoders(double power, int encoder){
        int encoderStart = robot.distance.sEncoders() ;
        robot.mDrive.drive(new Vector(power, 0));
        while(Math.abs(robot.distance.sEncoders() - encoderStart) < encoder){
            double temp = Math.abs(robot.distance.sEncoders() - encoderStart);
            if(encoder - temp < 1440) {
                robot.mDrive.drive(new Vector(0, ((encoder - temp)/1440)*power));
            }
        }
        robot.mDrive.stopAllMotors();
    }


    private void driveEncoders(double power, int encoder){
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.drive(new Vector(0, power));
        while(Math.abs(robot.distance.fEncoders() - encoderStart) < encoder){
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            if(encoder - temp < 1440) {
                robot.mDrive.drive(new Vector(0, ((encoder - temp)/1440)*power));
            }
        }
        robot.mDrive.stopAllMotors();
    }


    public void nin(int distDif) throws InterruptedException {
        //45 degrees is 500 ms
        turnEncoders(0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
    }

    public void nin() throws InterruptedException {
        //45 degrees is 500 ms
        turnEncoders(0.3, 400);
    }

    public void negnin(int distDif) throws InterruptedException {
        turnEncoders(-0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
    }

    public void negnin() throws InterruptedException {
        turnEncoders(-0.3, 400);
    }

    public void moveFound(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
            robot.grabber.toggleDown();
            wait(100);
            negnin();
            robot.grabber.toggleUp();
        } else {
            robot.grabber.toggleDown();
            wait(100);
            nin();
            robot.grabber.toggleUp();
        }
    }

    public void getBlock1Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
           /* negnin();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(-0.5, blockPos);*/
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            negnin();
            driveEncoders(0.5, specBlock1);
            nin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Blue");
            driveEncoders(-0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            nin();
            driveEncoders(0.5,processor.getEncoderAmount(1,Units.INCH));
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, 500);
            negnin();
            /*negnin();
            negnin();
            driveEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            negnin();
            negnin();*/
            driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        } else {
            /*nin();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, specBlock1);
            negnin();*/
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            driveEncoders(0.5, specBlock1);
            negnin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Red");
            driveEncoders(-0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(0.5,oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            /*negnin();
            negnin();
            driveEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            negnin();
            negnin();
            ?? think through and fix cause I copy/pasted from the blue side code*/
            driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        }

    }

    public void getBlock2Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
           /* negnin();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, specBlock2);
            nin();*/
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            negnin();
            driveEncoders(0.5, specBlock2);
            nin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Blue");
            driveEncoders(-0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            nin();
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            negnin();
            /*
            negnin();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            negnin();
            negnin();*/
            driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        } else {
            /*nin();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, specBlock2);
            negnin();*/
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            driveEncoders(0.5, specBlock1);
            negnin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Red");
            driveEncoders(-0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(0.5,oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            /*
            negnin();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(-0.5, blockPos);
            negnin();
            negnin();*/
            driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        }

    }

    public void getBlock3Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
           /* negnin();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, specBlock3);
            nin();*/
           driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            negnin();
            driveEncoders(0.5, specBlock3);
            nin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Blue");
            driveEncoders(-0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            nin();
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            negnin();
            /*negnin();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            negnin();
            negnin();*/
            driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        } else {
            /*nin();
            strafeEncoders(-0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, specBlock3);*/
            driveEncoders(0.5, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            driveEncoders(0.5, specBlock3);
            negnin();
            driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
            robot.blockIntakeServo.output();
            negnin();
            moveFound("Red");
            driveEncoders(-0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(0.5,oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-0.5, oneTile);
            nin();
            /*
            negnin();
            strafeEncoders(0.5, blockPos);
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.intake();
            strafeEncoders(-0.5, blockPos);
            negnin();
            negnin();*/
            driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
        }

    }

    public void getBlockBuild(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
            driveEncoders(0.5, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Blue");
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            nin();
            //run opencv
            if (layout1 = true) {
               /*negnin();
                strafeEncoders(0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();

                strafeEncoders(-0.5, blockPos);*/
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock1);
                nin();
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5,Units.INCH));
                /*nin();
                nin();
                strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, blockPos);
                nin();
                nin();*/
                driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
            else if (layout2 = true) {
                /*negnin();
                strafeEncoders(0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(-0.5, blockPos);*/
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock2);
                nin();
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5,Units.INCH));
                /*nin();
                nin();
                strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, blockPos);
                nin();
                nin();*/
                driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
            else if (layout3 = true) {
                /*negnin();
                strafeEncoders(0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(-0.5, blockPos);*/
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock3);
                nin();
                driveEncoders(0.5, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5, oneTile);
                negnin();
                driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5,Units.INCH));
                /*nin();
                nin();
                strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, blockPos);
                nin();
                nin();*/
                driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
        }
        else {
            driveEncoders(0.5, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Red");
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            negnin();
            //run opencv
            if (layout1 = true){
                /*strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, specBlock1);*/
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock1);
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5, Units.INCH));
                /*negnin();
                negnin();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(-0.5, processor.getEncoderAmount(4, Units.INCH));
                negnin();
                negnin();*/
                driveEncoders(0.5, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
            else if (layout2 = true){
                /*strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, specBlock1);*/
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock2);
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5, Units.INCH));
                 /*negnin();
                negnin();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(-0.5, processor.getEncoderAmount(4, Units.INCH));
                negnin();
                negnin();*/
                driveEncoders(0.5, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
            else if (layout3 = true){
                /*strafeEncoders(-0.5, blockPos);
                driveEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, specBlock1);*/
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock3);
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(0.5,oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-0.5,oneTile);
                nin();
                driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(5, Units.INCH));
                /*negnin();
                negnin();
                strafeEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intake();
                strafeEncoders(-0.5, processor.getEncoderAmount(4, Units.INCH));
                negnin();
                negnin();*/
                driveEncoders(0.5, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(0.5, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-0.5, processor.getEncoderAmount(6, Units.INCH));
            }
        }

    }




    @Override
    protected Robot buildRobot() {
        robot = new MainRobot(this);
        return robot;
    }
    
    Vector straight = new Vector(0, 1);
    Vector backwards = new Vector(0, -1);
    Vector left = new Vector(-1, 0);
    Vector right = new Vector(1, 0);
    int specBlock1 = processor.getEncoderAmount(10, Units. INCH);
    int specBlock2 = processor.getEncoderAmount(10, Units. INCH);
    int specBlock3 = processor.getEncoderAmount(10, Units. INCH);
    boolean layout1 = true;
    boolean layout2 = false;
    boolean layout3 = false;

    int i = 0;
    @Override
    public void main() throws InterruptedException {

        //robot.mDrive.reverseDirection();

        switch (robot.selector.autonomous) {
            case("Forward23in"):
                driveEncoders(0.5, processor.getEncoderAmount(23, Units.INCH));
                break;

            case ("Turn90"):
                nin();
                telemetry.update();
                break;


            case ("ParkOnBridge"):

                if (robot.selector.color.equals("Red")){

                    strafeEncoders(-0.5, processor.getEncoderAmount(2, Units.INCH));
                    driveEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
        }

                else if (robot.selector.color.equals("Blue")) {
                    strafeEncoders(0.5, processor.getEncoderAmount(2, Units.INCH));
                    driveEncoders(0.5, processor.getEncoderAmount(4, Units.INCH));
                }

                while(i<100) {
                    telemetry.addData("Encoders", robot.distance.fEncoders());
                    telemetry.update();
                    i++;
                }
                break;


            case ("MoveFoundationPark"):
                if (robot.selector.startPos.equals("Construction")) {
                    if (robot.selector.color.equals("Red")) {
                        driveEncoders(0.5, 800);
                        moveFound("Red");
                        telemetry.update();
                    }

                    if (robot.selector.color.equals("Blue")) {
                        driveEncoders(0.5, twoTile);
                        moveFound("Blue");
                        telemetry.update();
                    }
                }
                break;


            case ("MaxPoints"):
                if (robot.selector.startPos.equals("Resource")) {
                    //if the openCV returns the first layout
                    if (layout1 == true) {
                        // todo if (robot.openCV.layout.equals("getBlock1Res")){}
                        if (robot.selector.color.equals("Blue")) {
                            getBlock1Res("Blue");
                            telemetry.update();
                            break;
                        } else {
                            getBlock1Res("Red");
                            telemetry.update();
                            break;
                           /* if (true/*robot.skystoneDetector.layout == 1) {
                                getBlock1Res()
                            }
                            break;*/
                        }
                    }
                    /*if (robot.skystoneDeteector.layout == true) {
                        //   todo else if (robot.openCV.layout.equals("getBlock2Res")) {}
                        if (robot.selector.color.equals("Blue")) {
                            getBlock2Res("Blue");
                            telemetry.update();
                            break;
                        } else {
                            getBlock2Res("Red");
                            telemetry.update();
                            break;
                        }
                    }
                    if (layout3 == true) {
                        //     todo else if (robot.openCV.layout.equals("gerBlock3Res")) {}
                        if (robot.selector.color.equals("Blue")) {
                            getBlock3Res("Blue");
                            telemetry.update();
                            break;

                        } else {
                            getBlock3Res("Red");
                            telemetry.update();
                            break;
                        }
                    }
                }


                else if (robot.selector.startPos.equals("Build")) {
                    if (robot.selector.color.equals("Blue")) {
                        //run the generic start to the build autonomous, move the foundation and position in front of the blocks positioning to run opencv
                        getBlockBuild("Blue");
                        telemetry.update();
                        break;
                    }
                    else {
                        getBlockBuild("Red");
                        telemetry.update();
                        break;
                    }*/

                }

        }

    }




}
