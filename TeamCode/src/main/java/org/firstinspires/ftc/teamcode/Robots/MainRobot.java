package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.control.PIDController;
import com.SCHSRobotics.HAL9001.util.functional_interfaces.BiFunction;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.AutonomousSelectorSubsystemUsingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.ComputerVision.opencvSkystoneDetector_v2;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.CustomMechinumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.EncoderSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.FoundationGrabberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.IntakeSubSystemMotors;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.IntakeSubSystemServo;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.MarkerServoSubsystem;

import static java.lang.Math.PI;

public class MainRobot extends Robot {

    public CustomMechinumDrive mDrive;
    public FoundationGrabberSubsystem grabber;
    public EncoderSubsystem distance;
    public AutonomousSelectorSubsystemUsingConfig selector;
    public IntakeSubSystemServo blockIntakeServo;
    public IntakeSubSystemMotors blockIntakeMotors;
    public MarkerServoSubsystem markerServo;
    public opencvSkystoneDetector_v2 openCV;


    public MainRobot(OpMode opMode) {
        super(opMode);
        PIDController dsa = new PIDController(1.3, 0, 0, new BiFunction<Double,Double,Double>() {
            @Override
            public Double apply(Double target, Double current) {
                BiFunction<Double, Double, Double> mod = new BiFunction<Double, Double, Double>() {
                    @Override
                    public Double apply(Double x, Double m) {
                        return (x % m + m) % m;
                    }
                };

                double m = 2 * PI;

                //cw - ccw +
                double cw = mod.apply(mod.apply(current, m) - mod.apply(target, m), m);
                double ccw = -mod.apply(mod.apply(target, m) - mod.apply(current, m), m);

                return Math.abs(ccw) < Math.abs(cw) ? ccw : cw;
            }
        });
        dsa.deadband = PI / 90;

        startGui(new Button(1, Button.BooleanInputs.y));
        grabber = new FoundationGrabberSubsystem(this, "armL", "armR");
        mDrive = new CustomMechinumDrive(this, new CustomMechinumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1)
                .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setTurnPID(dsa)
                .setImuNumber(2));          //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");
        selector = new AutonomousSelectorSubsystemUsingConfig(this);
        blockIntakeServo = new IntakeSubSystemServo(this, "blockIntakeServo");
        blockIntakeMotors = new IntakeSubSystemMotors(this, "leftIntake", "rightIntake");
        distance = new EncoderSubsystem(this, "forwardEncoder", mDrive);
        markerServo = new MarkerServoSubsystem(this, "markerOutput");
        openCV = new opencvSkystoneDetector_v2(this);
        mDrive.getMotors()[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        putSubSystem("MechanumDrive", mDrive);
        putSubSystem("FoundationGrabber", grabber);
        putSubSystem("AutonomousSelector", selector);
        putSubSystem("ServoIntake", blockIntakeServo);
        putSubSystem("MotorIntake", blockIntakeMotors);
        putSubSystem("MarkerServo", markerServo);
        putSubSystem("OpenCV", openCV);
        putSubSystem("EncoderSubsystem", distance);
    }
}