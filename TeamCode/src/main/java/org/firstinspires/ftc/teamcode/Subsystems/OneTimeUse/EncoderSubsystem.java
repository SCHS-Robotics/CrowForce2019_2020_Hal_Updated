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
        robot.gui.addMenu("display", dMenu);
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
    public void init() throws InterruptedException {

    }

    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {
        forwardMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void handle() throws InterruptedException {
        dMenu.addData("Pos: ", forwardMotor.getCurrentPosition());
        dMenu.addData("Pos: ", strafeMotor.getCurrentPosition());
    }

    @Override
    public void stop() throws InterruptedException {

    }
}