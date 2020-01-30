package org.firstinspires.ftc.teamcode.OpModes.TellyOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MotorTest;
public class MotorTesting extends BaseTeleop {
    @Override
    protected Robot buildRobot() {
        return new MotorTest(this);
    }
}
