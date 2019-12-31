package org.firstinspires.ftc.teamcode.OpModes.TellyOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.ForwardTestRobot;

@StandAlone
@TeleOp(name = "ForwardTestOp")
public class ForwardTestOp extends BaseTeleop {
    private ForwardTestRobot robot;

    @Override
    protected Robot buildRobot() {
        robot = new ForwardTestRobot(this);
        return robot;
    }
}
