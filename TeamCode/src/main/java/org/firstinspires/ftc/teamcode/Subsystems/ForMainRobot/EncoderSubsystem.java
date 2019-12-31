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
    public void init()  {

        if(robot.usesGUI()) {
            dMenu = new DisplayMenu(robot.gui);
            robot.gui.addMenu("gui", dMenu);
        }
    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void handle()  {
        robot.telemetry.addData("Encoders", motor.getCurrentPosition());
        robot.telemetry.update();
    }

    @Override
    public void stop()  {

    }
}