package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderSubsystem extends SubSystem {

    DisplayMenu dMenu;
    DcMotor motor;
    MechanumDrive mDrive;

    public EncoderSubsystem(Robot r, String encoderMotor, MechanumDrive mDrive) {
        super(r);
        this.mDrive = mDrive;
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
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void handle()  {
        if(robot.gamepad1.a) {
            motor.setPower(.2);
        }
        else
        {
            motor.setPower(0);
        }
        robot.telemetry.addData("Encoders", motor.getCurrentPosition());
        robot.telemetry.addData("Roll: ", mDrive.getCurrentRoll());
        robot.telemetry.addData("Pich: ", mDrive.getCurrentPitch());

        robot.telemetry.addData("Angle: ", mDrive.getCurrentAngle());

        robot.telemetry.update();
    }

    @Override
    public void stop()  {

    }
}