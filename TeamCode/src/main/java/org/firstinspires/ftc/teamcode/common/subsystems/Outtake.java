package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;


public class Outtake extends SubsystemBase {
    public DcMotorEx flywheel;
    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {

        flywheel = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "Flywheel");

        flywheel.setVelocity(0);
        flywheel.setVelocityPIDFCoefficients(0, 0, 0, 0);


        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (updateAndPowerScheduler.outtakeUpdate) {
                if (!updateAndPowerScheduler.powerOuttake) {
                    flywheel.setMotorDisable();
                } else {
                    flywheel.setMotorEnable();
                }
                updateAndPowerScheduler.outtakeUpdate=false;
            }

            flywheel.setPower(1);
        });
    }

    public InstantCommand runFlywheel() {
        return new InstantCommand();
    }
}
