package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.EncoderTestRobot;

@StandAlone
@TeleOp(name="EncoderTest")
public class EncoderTest extends BaseTeleop {
    private EncoderTestRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new EncoderTestRobot(this);
        return robot;
    }

}