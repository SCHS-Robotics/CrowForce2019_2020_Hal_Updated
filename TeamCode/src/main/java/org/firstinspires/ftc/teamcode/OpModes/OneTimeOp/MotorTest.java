package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTest extends SubSystem {

    /**
     * Constructor for subsystem.
     *
     * @param robot - The robot the subsystem is contained within.
     */
    public MotorTest(Robot robot) {
        super(robot);
    }

    DcMotor motor1;

    @Override
    public void init() throws InterruptedException {
        motor1 = robot.hardwareMap.dcMotor.get("1");
    }

    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {

    }

    @Override
    public void handle() throws InterruptedException {
        if(robot.gamepad1.a){
            motor1.setPower(1);
        }
        else if(robot.gamepad1.b){
            motor1.setPower(.3);
        }
        else if(robot.gamepad1.y){
            motor1.setPower(-1);
        }
        else if(robot.gamepad1.x){
            motor1.setPower(-.3);
        }
        else{
            motor1.setPower(0);
        }
    }

    @Override
    public void stop() throws InterruptedException {

    }
}
