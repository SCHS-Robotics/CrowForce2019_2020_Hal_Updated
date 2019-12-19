package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class MotorTest extends Robot {
    /**
     * Constructor for robot.
     *
     * @param opMode - The opmode the robot is currently running.
     */
    public MotorTest(OpMode opMode) {
        super(opMode);
        super.putSubSystem("SubsystemSelector", new org.firstinspires.ftc.teamcode.OpModes.OneTimeOp.MotorTest(this));
    }
}
