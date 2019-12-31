package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.AutonomousSelectorSubsystemUsingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.FoundationGrabberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.IntakeSubSystemServo;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.MarkerServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.EncoderSubsystem;
import org.slf4j.Marker;

public class MainRobot extends Robot {

    public MechanumDrive mDrive;
    public FoundationGrabberSubsystem grabber;
    public EncoderSubsystem distance;
    public AutonomousSelectorSubsystemUsingConfig selector;
    public IntakeSubSystemServo blockIntakeServo;
    public MarkerServoSubsystem markerServo;
    //public opencvSkystoneDetector skystoneDetector;

    public MainRobot(OpMode opMode) {
        super(opMode);

        startGui(new Button(1, Button.BooleanInputs.y));
        grabber = new FoundationGrabberSubsystem(this, "arm");
        mDrive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setRevHubsInverted(true)
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1) .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
        .setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));        //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");
        selector = new AutonomousSelectorSubsystemUsingConfig(this);
        blockIntakeServo = new IntakeSubSystemServo(this, "blockIntake");
        distance = new EncoderSubsystem(this, "forwardEncoder", "strafeEncoder");
        markerServo = new MarkerServoSubsystem(this, "markerOutput");
        //skystoneDetector = new opencvSkystoneDetector();

        super.putSubSystem("SubsystemSelector", selector);
        super.putSubSystem("MainRobot", mDrive);
        super.putSubSystem("FoundationGrabber", grabber);
        super.putSubSystem("IntakeSubSystemMotors", blockIntakeServo);
        super.putSubSystem("EncoderValues", distance);
        super.putSubSystem("Marker Servo", markerServo);
    }
}