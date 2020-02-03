package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class FoundationGrabberSubsystem extends SubSystem {

    private final String ARMBUTTON = "armToggleButton";
    private final String ARMUPBUTTON = "armUpButton";
    Servo armL;
    CRServo arm;
    Servo armR;
    private final int UP = 1;
    private final int DOWN = 0;
    CustomizableGamepad gpad;
    Toggle toggle;
    DisplayMenu fMenu;

    public FoundationGrabberSubsystem(Robot r, String servoNameL, String servoNameR) {
        super(r);
        armL = r.hardwareMap.servo.get(servoNameL);
        armR = r.hardwareMap.servo.get(servoNameR);
        usesConfig = true;
        toggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    }


    @Override
    public void init()  {
        if (robot.usesGUI()) {
            fMenu = new DisplayMenu(robot.gui);
            robot.gui.addMenu("buttonData", fMenu);
        }
    }


    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        gpad = robot.pullControls(this);
    }

    @Override
    public void handle()  {
        toggle.updateToggle(gpad.getBooleanInput("armToggleButton"));
        if (toggle.getCurrentState()) {
            armL.setPosition(-1);
            armR.setPosition(-1);
        }
         else {
            armL.setPosition(1);
            armR.setPosition(1);
        }


        //for toggle
        //toggle.updateToggle(gpad.getBooleanInput(ARMUPBUTTON));


    }

    @Override
    public void stop()  {

    }


    @Override
    public void initVars() { super.initVars(); }

    public void toggleDown() {
        long startTime = System.currentTimeMillis();
        armL.setPosition(1);
        armR.setPosition(1);
    }

    public void toggleUp() {
        long startTime = System.currentTimeMillis();
        armL.setPosition(-1);
        armR.setPosition(-.75);


    }
    
    public void toggleOff(){arm.setPower(.5);}

    public void pullUp(int timeMs)  {
        toggleUp();

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < timeMs && robot.opModeIsActive()) {

        }

        toggleOff();
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("armToggleButton", Button.BooleanInputs.a,2),
        };
    }
}