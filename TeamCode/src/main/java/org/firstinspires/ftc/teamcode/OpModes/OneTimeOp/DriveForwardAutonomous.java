package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.ForwardTestRobot;

@StandAlone
@Autonomous (name="ForwardAutonomous")
public class DriveForwardAutonomous extends BaseAutonomous {
    private ForwardTestRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new ForwardTestRobot(this);
        return robot;
    }

    @Override
    public void main() throws InterruptedException{
        robot.drive.driveTime(new Vector(0, .07), 3000);
    }
}
