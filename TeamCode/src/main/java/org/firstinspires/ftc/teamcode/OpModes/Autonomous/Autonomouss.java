package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.SCHSRobotics.HAL9001.util.math.EncoderToDistanceProcessor;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Robots.MainRobot;


@StandAlone
@Autonomous(name = "Autonomous")
public class Autonomouss extends BaseAutonomous {
    private MainRobot robot;
    private EncoderToDistanceProcessor processor = new EncoderToDistanceProcessor(2.5, Units.INCH);
   // list of common encoder
  //  distances:1tile
    int oneTile = processor.getEncoderAmount(7, Units.INCH);
    //2tiles
    int twoTile = processor.getEncoderAmount(25, Units.INCH);
    //2.5

   // tiles(right before of intake)

    int blockPos = processor.getEncoderAmount(7, Units.INCH);

    private void turnEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.turn(power);
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            if (encoder - temp < 1440) {
                robot.mDrive.turn((((encoder - temp) / 1440) * power) + .02);
            }
        }
        robot.mDrive.stopAllMotors();
    }

    private void strafeEncoders(double power, int encoder) {
        int encoderStart = robot.distance.sEncoders();
        robot.mDrive.drive(new Vector(power, 0));
        while (Math.abs(robot.distance.sEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.sEncoders() - encoderStart);
            telemetry.addData("Strafe Encoders:", robot.distance.sEncoders());
            telemetry.update();
            if (encoder - temp < 1440) {
                robot.mDrive.drive(new Vector((((encoder - temp) / 1440) * power + .02), 0));
            }
        }
        robot.mDrive.stopAllMotors();
    }

    private void stopTime(){
        long startTime = System.currentTimeMillis();
        robot.mDrive.stopAllMotors();

        while (System.currentTimeMillis() - startTime < 1000) {
        }
    }
    
    private void driveEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.drive(new Vector(0, power));
        telemetry.addData("Before: ", power);
        telemetry.update();
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            telemetry.addData("Forward Encoders:", robot.distance.fEncoders());
            telemetry.addData("Before: ", power);
            if (encoder - temp < 1440) {
                robot.mDrive.drive(new Vector(0, (((encoder - temp) / 1440) * power) + .02));
                telemetry.addData("After: ", ((encoder - temp) / 1440) * power);
            }
            telemetry.update();
        }
        robot.mDrive.stopAllMotors();
    }

    public void nin(int distDif) throws InterruptedException {
      //  45 degrees is 500 ms
        turnEncoders(0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
    }

    public void nin() throws InterruptedException {
        //45 degrees is 500 ms
    turnEncoders (0.3, 400);
    }

    public void negnin(int distDif) throws InterruptedException {
        turnEncoders(-0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
    }

    public void negnin() throws InterruptedException {
        turnEncoders(-0.3, 400);
    }

    public void moveFound(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Red")) {
            robot.grabber.toggleDown();
            //robot.mDrive.driveTime(new Vector(0,0.15), 2700);
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < 4000) {
                robot.mDrive.setBotLeftPower(0.2);
                robot.mDrive.setBotRightPower(.2);
                robot.mDrive.setTopLeftPower(.2);
                robot.mDrive.setTopRightPower(.2);

            }
            robot.mDrive.stopAllMotors();
            robot.grabber.toggleUp();
        } else {
            robot.grabber.toggleDown();
           // robot.mDrive.driveTime(new Vector(0,0.15), 2700);
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < 2700) {
                robot.mDrive.setBotLeftPower(0.2);
                robot.mDrive.setBotRightPower(.2);
                robot.mDrive.setTopLeftPower(.2);
                robot.mDrive.setTopRightPower(.2);

            }
            robot.mDrive.stopAllMotors();
            robot.grabber.toggleUp();
        }
    }

    public void getBlock1Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue") /*&& skystoneInd = 1;*/) { /* negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos);*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin();
            driveEncoders(.15, specBlock1);
            negnin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Blue");
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin(); /*negnin(); negnin(); driveEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); negnin(); negnin();*/
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        } else { /*nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); driveEncoders(.15, specBlock1); negnin();*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            nin();
            driveEncoders(.15, specBlock1);
            nin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Red");
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            nin(); /*negnin(); negnin(); driveEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); negnin(); negnin(); ?? think through and fix cause I copy/pasted from the blue side code*/
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        }
    }

    public void getBlock2Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue") /*&& skystoneInd = 2*/) { /* negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos); driveEncoders(.15, specBlock2); nin();*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin();
            driveEncoders(.15, specBlock1);
            negnin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Blue");
            driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile); /* negnin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); negnin(); negnin();*/
            driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        } else { /*nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); driveEncoders(.15, specBlock2); negnin();*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            nin();
            driveEncoders(.15, specBlock2);
            nin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Red");
            driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            nin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin(); /* negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos); negnin(); negnin();*/
            driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        }
    }

    public void getBlock3Res(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) { /* negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos); driveEncoders(.15, specBlock3); nin();*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin();
            driveEncoders(.15, specBlock1);
            negnin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Blue");
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile); /*negnin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); negnin(); negnin();*/
            driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        } else { /*nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); driveEncoders(.15, specBlock3);*/
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin();
            driveEncoders(.15, specBlock1);
            negnin();
            robot.blockIntakeServo.output();
            driveEncoders(-.15, twoTile);
            moveFound("Blue");
            driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
            negnin();
            driveEncoders(.15, oneTile);
            robot.blockIntakeServo.intake();
            driveEncoders(-.15, oneTile);
            negnin(); /* negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos); negnin(); negnin();*/
            driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
            driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
            robot.blockIntakeServo.output();
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
        }
    }

    public void getBlockBuild(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
            driveEncoders(.15, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Blue");
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            nin();
            //run opencv
            if (layout1 = true) { /*negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock1);
                nin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); nin(); nin();*/
                driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout2 = true) { /*negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock2);
                nin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); nin(); nin();*/
                driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout3 = true) { /*negnin(); strafeEncoders(.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, blockPos);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock3);
                nin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                negnin();
                driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, blockPos); nin(); nin();*/
                driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            }
        } else {
            driveEncoders(.15, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Red");
            driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            negnin();
            //run opencv
            if (layout1 = true) { /*strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, specBlock1);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock1);
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();*/
                driveEncoders(.15, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout2 = true) { /*strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, specBlock1);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock2);
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();*/
                driveEncoders(.15, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout3 = true) { /*strafeEncoders(-.15, blockPos); driveEncoders(.15, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, specBlock1);*/
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock3);
                robot.blockIntakeServo.output();
                driveEncoders(-.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(.15, oneTile);
                robot.blockIntakeServo.intake();
                driveEncoders(-.15, oneTile);
                nin();
                driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(.15, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intake(); strafeEncoders(-.15, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();*/
                driveEncoders(.15, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(.15, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.output();
                driveEncoders(-.15, processor.getEncoderAmount(6, Units.INCH));
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
    int specBlock1 = processor.getEncoderAmount(10, Units.INCH);
    int specBlock2 = processor.getEncoderAmount(10, Units.INCH);
    int specBlock3 = processor.getEncoderAmount(10, Units.INCH);
    boolean layout1 = true;
    boolean layout2 = false;
    boolean layout3 = false;
    boolean once = true;
    int i = 0;
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor botRight;
    DcMotor botLeft;
    long startTime = System.currentTimeMillis();

    @Override
    public void main() throws InterruptedException {
        if(once){
            hardwareMap.dcMotor.get("topLeft").setDirection(DcMotor.Direction.REVERSE);
            robot.markerServo.MarkerServo.setPosition(.5);
            once = false;
        }
        telemetry.setAutoClear(true);
        switch (robot.selector.autonomous) {
            case ("Forward23in"):
                //driveEncoders(0.3, processor.getEncoderAmount(10, Units.INCH));
                while(System.currentTimeMillis() - startTime < 1200) {
                    robot.mDrive.setBotLeftPower(0.2);
                    robot.mDrive.setBotRightPower(.2);
                    robot.mDrive.setTopLeftPower(.2);
                    robot.mDrive.setTopRightPower(.2);

                }
                stopTime();
                break;
            case ("Turn90"):
                nin();
                telemetry.update();
                break;
            case ("ParkOnBridge"):
                if (robot.selector.color.equals("Red")) {
                    driveEncoders(0.15, oneTile);
                } else if (robot.selector.color.equals("Blue")) {
                    driveEncoders(0.15, oneTile);
                }
                break;
            case ("MoveFoundationPark"):
                    if (robot.selector.color.equals("Red")) {
                       /* strafeEncoders(-.2, oneTile);
                        stopTime();
                        driveEncoders(-.15, twoTile);
                        stopTime();*/
                        //robot.mDrive.driveTime(new Vector(0.15,0), 1500);

                        while(System.currentTimeMillis() - startTime < 1200) {
                            robot.mDrive.setBotLeftPower(0.2);
                            robot.mDrive.setBotRightPower(-.2);
                            robot.mDrive.setTopLeftPower(-.2);
                            robot.mDrive.setTopRightPower(.2);

                        }
                        stopTime();
                        startTime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - startTime < 1000) {
                            robot.mDrive.setBotLeftPower(-0.05);
                            robot.mDrive.setBotRightPower(-.05);
                            robot.mDrive.setTopLeftPower(-.05);
                            robot.mDrive.setTopRightPower(-.05);

                        }
                        robot.mDrive.stopAllMotors();
                        //robot.mDrive.driveTime(new Vector(0,-0.15), 2500);
                        stopTime();
                        moveFound("Red");

                        stopTime();
                        robot.mDrive.driveTime( new Vector(0.15,0), 3000);
                        //strafeEncoders(.2,processor.getEncoderAmount(33, Units.INCH));
                    } else if (robot.selector.color.equals("Blue")) {
                        long startTime = System.currentTimeMillis();
                       /* strafeEncoders(.2,oneTile);
                        stopTime();
                        driveEncoders(-.15, twoTile);
                        stopTime();*/
                       // robot.mDrive.driveTime(new Vector(0.15,0), 2500);
                        while(System.currentTimeMillis() - startTime < 1200) {
                            robot.mDrive.setBotLeftPower(-0.2);
                            robot.mDrive.setBotRightPower(.2);
                            robot.mDrive.setTopLeftPower(.2);
                            robot.mDrive.setTopRightPower(-.2);

                        }
                        robot.mDrive.stopAllMotors();
                        stopTime();
                        //robot.mDrive.driveTime(new Vector(0,-0.15), 2500);
                        startTime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - startTime < 3000) {
                            robot.mDrive.setBotLeftPower(-0.02);
                            robot.mDrive.setBotRightPower(-.1);
                            robot.mDrive.setTopLeftPower(-0.02);
                            robot.mDrive.setTopRightPower(-.1);
                        }
                        robot.mDrive.stopAllMotors();
                        stopTime();
                        moveFound("Red");
                        stopTime();
                        robot.mDrive.driveTime( new Vector(-0.15,0), 3000);
                        //strafeEncoders(-.2,processor.getEncoderAmount(33, Units.INCH));
                        telemetry.update();
                    }

                break;
            case ("MaxPoints"):
                if (robot.selector.startPos.equals("Resource")) {
                    //if the openCV returns the first layout
                    // if (layout1 == true) {
                        //todo if (robot.openCV.layout.equals("getBlock1Res")) {
                        }
                        if (robot.selector.color.equals("Blue")) {
                            getBlock1Res("Blue");
                            telemetry.update();
                            break;
                        } else {
                            getBlock1Res("Red");
                            telemetry.update();
                            break; /* if (true/*robot.skystoneDetector.layout == 1) { getBlock1Res() } break;*/
                        }
                    } /*if (robot.skystoneDeteector.layout == true) {    todo else if (robot.openCV.layout.equals("getBlock2Res")) {} if (robot.selector.color.equals("Blue")) { getBlock2Res("Blue"); telemetry.update(); break; } else { getBlock2Res("Red"); telemetry.update(); break; } } if (layout3 == true) {      todo else if (robot.openCV.layout.equals("gerBlock3Res")) {} if (robot.selector.color.equals("Blue")) { getBlock3Res("Blue"); telemetry.update(); break; } else { getBlock3Res("Red"); telemetry.update(); break; } } } else if (robot.selector.startPos.equals("Build")) { if (robot.selector.color.equals("Blue")) { run the generic start to the build autonomous, move the foundation and position in front of the blocks positioning to run opencv getBlockBuild("Blue"); telemetry.update(); break; } else { getBlockBuild("Red"); telemetry.update(); break; }*/
                }
        }
    //}
//}