package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderSubsystem extends SubSystem {

    DisplayMenu dMenu;
    DcMotor motor;

    public EncoderSubsystem(Robot r, String encoderMotor) {
        super(r);
        motor = robot.hardwareMap.dcMotor.get(encoderMotor);
    }

    @Override
    public void init() throws InterruptedException {

        if(robot.usesGUI()) {
            dMenu = new DisplayMenu(robot.gui);
            robot.gui.addMenu("gui", dMenu);
        }
    }

    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void handle() throws InterruptedException {
        dMenu.addData("Encoders", motor.getCurrentPosition());
    }

    @Override
    public void stop() throws InterruptedException {

    }
}