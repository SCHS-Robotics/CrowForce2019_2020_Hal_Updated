package org.firstinspires.ftc.teamcode.calibration_programs.PIDTuner;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * A robot object containing a subsystem used to tune turn-to-angle PID controllers.
 */
public class AnglePIDTunerBotCustom extends Robot {


    /**
     * Constructor for AnglePIDTunerBot
     *
     * @param opMode - The opmode the robot is running.
     */
    public AnglePIDTunerBotCustom(OpMode opMode) {
        super(opMode);
        startGui(new Button(1, Button.BooleanInputs.b));
        putSubSystem("PID Tuner", new AnglePIDTunerSystemCustom(this, Math.PI));
    }
}
