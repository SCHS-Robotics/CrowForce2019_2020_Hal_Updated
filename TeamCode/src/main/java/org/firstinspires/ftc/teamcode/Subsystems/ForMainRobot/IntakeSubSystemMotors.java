package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;

import com.SCHSRobotics.HAL9001.system.menus.DisplayMenu;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubSystemMotors extends SubSystem {
    private CustomizableGamepad inputs;
    DcMotor InL;
    DcMotor InR;

    private final String INBUTTON = "InButton", OUTBUTTON = "OutButton";

    public IntakeSubSystemMotors(Robot r, String inl, String inr) {
        super(r);
        InL = robot.hardwareMap.dcMotor.get(inl);
        InR = robot.hardwareMap.dcMotor.get(inr);
        usesConfig = true;
    }

    CustomizableGamepad gpad;
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
        dMenu.addData("InButton", gpad.getBooleanInput(INBUTTON));
        dMenu.addData("OutButton", gpad.getBooleanInput(OUTBUTTON));
        if (gpad.getBooleanInput(INBUTTON) && gpad.getBooleanInput(OUTBUTTON)) {
            InL.setPower(0);
            InR.setPower(0);
        } else if (gpad.getBooleanInput(INBUTTON)) {
            InL.setPower(-1);
            InR.setPower(1);
        } else if (gpad.getBooleanInput(OUTBUTTON)) {
            InL.setPower(1);
            InR.setPower(-1);
        } else {
            InL.setPower(0);
            InR.setPower(0);
        }
    }

    @Override
    public void stop () throws InterruptedException {

    }

    public void intake(double ms) {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= ms) {
            InL.setPower(-1);
            InR.setPower(1);
        }
        InL.setPower(0);
        InR.setPower(0);
    }

    public void output(double ms) {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime <= ms) {
            InL.setPower(1);
            InR.setPower(-1);
        }
        InL.setPower(0);
        InR.setPower(0);
    }

    @Override
    protected void initVars() {
        super.initVars();

    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("InButton", Button.BooleanInputs.right_bumper),
                new ConfigParam("OutButton", Button.BooleanInputs.left_bumper)
        };
    }
}
