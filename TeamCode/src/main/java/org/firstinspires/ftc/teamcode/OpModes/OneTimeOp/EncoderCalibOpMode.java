package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.EncoderDistanceCalibRobot;

@TeleOp
public class EncoderCalibOpMode extends BaseTeleop {

    EncoderDistanceCalibRobot robot;

    @Override
    protected Robot buildRobot() {
        robot = new EncoderDistanceCalibRobot(this);
        return robot;
    }

}
