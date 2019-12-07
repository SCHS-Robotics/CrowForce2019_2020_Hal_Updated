package org.firstinspires.ftc.teamcode.OpModes.TellyOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robots.ForwardTestRobot;
import org.firstinspires.ftc.teamcode.Robots.MainRobot;

@StandAlone
@TeleOp (name = "Main Teleop")
public class MainTeleop extends BaseTeleop {
    private MainRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new MainRobot(this);
        return robot;
    }
}