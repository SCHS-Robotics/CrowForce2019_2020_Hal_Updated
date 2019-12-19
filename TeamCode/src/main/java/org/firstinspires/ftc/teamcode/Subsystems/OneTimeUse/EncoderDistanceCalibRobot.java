package org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.calib.EncoderDistanceCalib;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.BetterEncoderCalibProgram;
import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.EncoderSubsystem;

public class EncoderDistanceCalibRobot extends Robot {

    //EncoderDistanceCalib subsystem
    private BetterEncoderCalibProgram EncoderDist;
    EncoderSubsystem EncoderNum;

    public EncoderDistanceCalibRobot(OpMode opMode) {
        super(opMode);
        startGui(new Button(1, Button.BooleanInputs.b));
        EncoderDist = new BetterEncoderCalibProgram(this, BetterEncoderCalibProgram.DriveTrain.MECHANUM_DRIVE, Units.INCH, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight"), new Button(1, Button.BooleanInputs.a));
        EncoderNum = new EncoderSubsystem(this, "topRight", "topLeft");

        super.putSubSystem("EncoderInput", EncoderNum);
        super.putSubSystem("EncoderDistanceCalibTest", EncoderDist);
    }
}
