package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.AutonomousSelectorSubsystemUsingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.FoundationGrabberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.IntakeSubSystemServo;

import java.util.Base64;

public class EncoderTestRobot extends Robot {


    public EncoderTestRobot EncoderTestRobot;


    public EncoderTestRobot(OpMode opMode) {
        super(opMode);

        encoderTestRobot = new EncoderTestRobot(this);
        //skystoneDetector = new opencvSkystoneDetector();


        super.putSubSystem("encoderTestRobot", encoderTestRobot);
    }
}