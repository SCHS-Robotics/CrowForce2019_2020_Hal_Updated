package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class NewIntakeSubSystemServo extends SubSystem {
    private CustomizableGamepad inputs;
    Servo IntakeServo;
    private final int DOWN = 1;
    private final double UP = 0.75;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, true);

    static final String INTAKEBUTTON = "IntakeButton";

    public NewIntakeSubSystemServo(Robot r, String servo) {
        super(r);
        IntakeServo = robot.hardwareMap.servo.get(servo);
        usesConfig = true;
    }

    private DisplayMenu dMenu;

    @Override
    public void init() throws InterruptedException {
        if(robot.usesGUI()) {
            dMenu = new DisplayMenu(robot.gui);
            robot.gui.addMenu("buttonData", dMenu);
        }
    }

    @Override
    public void init_loop() throws InterruptedException {

    }

    @Override
    public void start() throws InterruptedException {
        inputs = robot.pullControls(this);
    }
    @Override
    public void handle () throws InterruptedException {
        dMenu.addData("IntakeButton", inputs.getBooleanInput(INTAKEBUTTON));
        toggle.updateToggle(inputs.getBooleanInput(INTAKEBUTTON));
        if (toggle.getCurrentState()) {
            IntakeServo.setPosition(DOWN);
        }
        else {
            IntakeServo.setPosition(UP);
        }
    }

    @Override
    public void stop () throws InterruptedException {

    }

    public void intake () {
        IntakeServo.setPosition(DOWN);
    }


    public void output() {
        IntakeServo.setPosition(UP);
    }




    @Override
    protected void initVars() {
        super.initVars();

    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(INTAKEBUTTON, Button.BooleanInputs.right_bumper),
        };
    }
}

