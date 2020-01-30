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

public class IntakeSubSystemServo extends SubSystem {
    private CustomizableGamepad inputs;
    Servo IntakeServo;
    private final int DOWN = 0;
    private final double UP = 0.25;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, true);


    static final String INTAKEBUTTON = "IntakeButton";

    public IntakeSubSystemServo(Robot r, String servo) {
        super(r);
        IntakeServo = robot.hardwareMap.servo.get(servo);
        usesConfig = true;
    }


    @Override
    public void init()  {
        IntakeServo.setPosition(UP);
    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        inputs = robot.pullControls(this);
    }
    @Override
    public void handle ()  {
        toggle.updateToggle(inputs.getBooleanInput(INTAKEBUTTON));
        if (toggle.getCurrentState()) {
            IntakeServo.setPosition(DOWN);
        }
        else {
            IntakeServo.setPosition(UP);
        }

    }

    @Override
    public void stop ()  {

    }

    public void intakeDown () {
        IntakeServo.setPosition(DOWN);
    }


    public void intakeUp() {
            IntakeServo.setPosition(UP);
        }




    @Override
    protected void initVars() {
        super.initVars();

    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(INTAKEBUTTON, Button.BooleanInputs.x,2)
        };
    }
}

