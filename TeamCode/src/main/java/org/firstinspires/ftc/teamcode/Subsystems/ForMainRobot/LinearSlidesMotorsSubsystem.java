package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlidesMotorsSubsystem extends SubSystem {
    CustomizableGamepad input;
    DcMotor botM;
    DcMotor topM;
    public LinearSlidesMotorsSubsystem(Robot r, String botMotor, String topMotor) {
        super(r);
        botM = robot.hardwareMap.dcMotor.get(botMotor);
        topM = robot.hardwareMap.dcMotor.get(topMotor);
    }
    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        input = robot.pullControls(this);
    }

    @Override
    public void handle() {
        if(input.getBooleanInput("DownButton")) {
            botM.setPower(1);
            topM.setPower(-1);
        }
        if(input.getBooleanInput("UpButton")) {
            botM.setPower(-1);
            topM.setPower(1);
        }
    }

    @Override
    public void stop() {

    }

    @Override
    protected void initVars() {
        super.initVars();
    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("UpButton", Button.BooleanInputs.dpad_up,2),
                new ConfigParam("DownButton", Button.BooleanInputs.dpad_down,2)
        };
    }
}
