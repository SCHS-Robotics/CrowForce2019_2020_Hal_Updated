package org.firstinspires.ftc.teamcode.Robots;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.calib.AnglePIDTunerSystem;
import com.SCHSRobotics.HAL9001.util.control.PIDController;
import com.SCHSRobotics.HAL9001.util.functional_interfaces.BiFunction;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot.CustomMechinumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse.CustomPidTuner;

import static java.lang.Math.PI;

public class PID_TunerRobot extends Robot {

    CustomPidTuner PIDSubsystem;

    public PID_TunerRobot(OpMode opMode) {

        super(opMode);
        PIDController dsa = new PIDController(0, 0, 0, new BiFunction<Double,Double,Double>() {
            @Override
            public Double apply(Double target, Double current) {
                BiFunction<Double, Double, Double> mod = new BiFunction<Double, Double, Double>() {
                    @Override
                    public Double apply(Double x, Double m) {
                        return (x % m + m) % m;
                    }
                };

                double m = 2 * PI;

                //cw - ccw +
                double cw = mod.apply(mod.apply(current, m) - mod.apply(target, m), m);
                double ccw = -mod.apply(mod.apply(target, m) - mod.apply(current, m), m);

                return Math.abs(ccw) < Math.abs(cw) ? ccw : cw;
            }
        });
        enableViewport(new Button(1, Button.BooleanInputs.noButton));
        dsa.setDeadband(PI/90);
        PIDSubsystem = new CustomPidTuner(this, new CustomMechinumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1) .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a)),
                dsa, PI/2, AngleUnit.RADIANS);
        putSubSystem("D", PIDSubsystem);
    }

}
