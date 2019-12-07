package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.AutonomousSelectorSubsystemUsingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.FoundationGrabberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.IntakeSubSystemServo;
import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.EncoderSubsystem;

public class MainRobot extends Robot {

    public MechanumDrive mDrive;
    public FoundationGrabberSubsystem grabber;
    public EncoderSubsystem distance;
    public AutonomousSelectorSubsystemUsingConfig selector;
    public IntakeSubSystemServo blockIntakeServo;
    //public opencvSkystoneDetector skystoneDetector;

    public MainRobot(OpMode opMode) {
        super(opMode);


        grabber = new FoundationGrabberSubsystem(this, "arm");
        mDrive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight"));
        //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");
        selector = new AutonomousSelectorSubsystemUsingConfig(this);
        blockIntakeServo = new IntakeSubSystemServo(this, "blockIntake");
        distance = new EncoderSubsystem(this, "topRight", "topLeft");
        //skystoneDetector = new opencvSkystoneDetector();

        super.putSubSystem("SubsystemSelector", selector);
        super.putSubSystem("MainRobot", mDrive);
        super.putSubSystem("FoundationGrabber", grabber);
        super.putSubSystem("IntakeSubSystemMotors", blockIntakeServo);
        super.putSubSystem("EncoderValues", distance);
    }
}