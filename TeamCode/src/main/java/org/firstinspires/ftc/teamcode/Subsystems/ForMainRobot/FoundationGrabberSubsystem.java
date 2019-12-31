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

import static java.lang.Thread.sleep;

public class FoundationGrabberSubsystem extends SubSystem {

    private final String ARMBUTTON = "armToggleButton";
    private final String ARMUPBUTTON = "armUpButton";
    CRServo arm;
    private final int UP = 1;
    private final int DOWN = 0;
    CustomizableGamepad gpad;
    Toggle toggle;
    DisplayMenu fMenu;

    public FoundationGrabberSubsystem(Robot r, String servoName) {
        super(r);
        arm = r.hardwareMap.crservo.get(servoName);
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
        if(gpad.getBooleanInput(ARMUPBUTTON)) {
            arm.setPower(-1);
        }

        else if (toggle.getCurrentState()) {
            arm.setPower(1);
        }
         else {
            arm.setPower(0);
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
        arm.setPower(1);

        while (System.currentTimeMillis() - startTime < 470) {
            robot.telemetry.addData("death","");
            arm.setPower(1);
            robot.telemetry.update();
        }
        arm.setPower(0);
    }

    public void toggleUp() {
        long startTime = System.currentTimeMillis();
        arm.setPower(-1);

        while (System.currentTimeMillis() - startTime < 470) {
            robot.telemetry.addData("death","");
            arm.setPower(-1);
            robot.telemetry.update();
        }
        arm.setPower(0);
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
                new ConfigParam("armToggleButton", Button.BooleanInputs.b,2),
                new ConfigParam("armUpButton", Button.BooleanInputs.a,2)
        };
    }
}