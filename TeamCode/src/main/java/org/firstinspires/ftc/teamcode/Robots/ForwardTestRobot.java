package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.OneTimeOp.ForwardTestOp;
import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.EncoderSubsystem;

public class ForwardTestRobot extends Robot {


    public EncoderSubsystem encoder;
    public MechanumDrive drive;


    public ForwardTestRobot(OpMode opMode) {
        super(opMode);
        startGui(new Button(1, Button.BooleanInputs.a));

        drive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight"));
        encoder = new EncoderSubsystem(this, "forwardEncoder", "strafeEncoder");
        /*.setDriveStick(new Button(1, Button.VectorInputs.left_stick))
        .setTurnStick(new Button(1, Button.DoubleInput` s.right_stick_x))
        .setSpeedModeMultiplier(.25));*/
        //encoder = new EncoderSubsystem(this, "1");
        putSubSystem("Mechanum Drive", drive);
        putSubSystem("EncoderSubsytem", encoder);

    }
}
