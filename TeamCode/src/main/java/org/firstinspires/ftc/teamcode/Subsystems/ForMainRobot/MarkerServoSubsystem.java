package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerServoSubsystem extends SubSystem {
    CustomizableGamepad inputs;
    public Servo MarkerServo;
    private final int DOWN = 1;
    private final double UP = 0.5;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    static final String MARKERBUTTON = "MarkerButton";

    public MarkerServoSubsystem(Robot r, String markerServo) {
        super(r);
        MarkerServo = robot.hardwareMap.servo.get(markerServo);
        usesConfig = true;
    }

    @Override
    public void init() throws InterruptedException {
    }

    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {
        inputs = robot.pullControls(this);
    }

    @Override
    public void handle() throws InterruptedException {
        robot.telemetry.addData("MarkerButton", inputs.getBooleanInput(MARKERBUTTON));
        robot.telemetry.addData("Toggle State", toggle.getCurrentState());
        robot.telemetry.update();
        toggle.updateToggle(inputs.getBooleanInput(MARKERBUTTON));
        if (toggle.getCurrentState()) {
            MarkerServo.setPosition(1);
        }
        else {
            MarkerServo.setPosition(.5);
        }
    }

    @Override
    public void stop() throws InterruptedException {

    }

    @Override
    protected void initVars() {
        super.initVars();

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(MARKERBUTTON, Button.BooleanInputs.y, 2)
        };
    }
}
