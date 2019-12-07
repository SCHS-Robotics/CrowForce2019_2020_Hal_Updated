package org.firstinspires.ftc.teamcode.calibration_programs.PIDTuner;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.calibration_programs.PIDTuner.AnglePIDTunerBotCustom;


/**
 * A simple teleop program for tuning turn-to-angle PID controllers.
 */
@StandAlone
@TeleOp(name = "Angle PID Tuner Custom", group = "Calibration")
public class AnglePIDTunerCustom extends BaseTeleop {

    //The robot being used.
    private AnglePIDTunerBotCustom robot;

    @Override
    public Robot buildRobot() {
        robot = new AnglePIDTunerBotCustom(this);
        return robot;
    }
}
