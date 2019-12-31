package org.firstinspires.ftc.teamcode.OpModes.TellyOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.PID_TunerRobot;

@StandAlone
@TeleOp(name = "PID Tuner")
public class PID_Tuner extends BaseTeleop {

    PID_TunerRobot asd;
    @Override
    protected Robot buildRobot() {
        asd = new PID_TunerRobot(this);
        return asd;
    }
}
