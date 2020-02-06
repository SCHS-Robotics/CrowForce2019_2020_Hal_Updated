package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class LinerSlidesServosSubsystem extends SubSystem {
    CustomizableGamepad inputs;
    Servo servoL;
    Servo servoR;
    public LinerSlidesServosSubsystem(Robot r, String servoLeft, String servoRight) {
        super(r);
        servoL = robot.hardwareMap.servo.get(servoLeft);
        servoR = robot.hardwareMap.servo.get(servoRight);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        inputs = robot.pullControls(this);
        servoR.setPosition(0);
        servoR.setPosition(0);
    }

    @Override
    public void handle() {
        //todo implement toggles
        if(inputs.getBooleanInput("SetNeutralButton")) {
            servoR.setPosition(0);
            servoR.setPosition(0);
        }
        if(inputs.getBooleanInput("SetUpButton")) {
            servoR.setPosition(0);
            servoR.setPosition(0);
        }
        if(inputs.getBooleanInput("SetDownButton")) {
            servoR.setPosition(0);
            servoR.setPosition(0);
        }
    }

    @Override
    public void stop() {

    }
    /*
    @Override
    protected void initVars() {
        super.initVars();
    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("SetNeutralButton", Button.BooleanInputs.right_bumper,2),
                new ConfigParam("SetUpButton", Button.BooleanInputs.left_bumper,2),
                new ConfigParam("SetDownlButton", Button.BooleanInputs.right_bumper,2),
        };
    }

     */
}
