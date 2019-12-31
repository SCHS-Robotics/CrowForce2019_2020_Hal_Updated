package org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.DcMotor;


public class EncoderSubsystem extends SubSystem {

    DisplayMenu dMenu;
    public DcMotor strafeMotor;
    public DcMotor forwardMotor;

    public EncoderSubsystem(Robot r, String fMotor, String sMotor) {
        super(r);
        dMenu = new DisplayMenu(robot.gui);
        strafeMotor = robot.hardwareMap.dcMotor.get("strafeEncoder");
        forwardMotor = robot.hardwareMap.dcMotor.get("forwardEncoder");

        forwardMotor = robot.hardwareMap.dcMotor.get(fMotor);
        strafeMotor = robot.hardwareMap.dcMotor.get(sMotor);
    }

    public int fEncoders() {
        return forwardMotor.getCurrentPosition();
    }

    public int sEncoders() {
        return strafeMotor.getCurrentPosition();
    }

    @Override
    public void init(){

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start(){
        forwardMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void handle() {
        robot.telemetry.addData("forward", fEncoders());
        robot.telemetry.addData("strafe", sEncoders());
        robot.telemetry.update();
    }

    @Override
    public void stop() {

    }
}