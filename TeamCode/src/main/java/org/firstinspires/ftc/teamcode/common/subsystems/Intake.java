package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

import java.util.HashMap;
@Configurable
public class Intake extends SubsystemBase {
    MotorEx intakeSpinner;

    Globals globals;

    UpdateAndPowerScheduler updateAndPowerScheduler;

    public static double intakeSpinnerPower = 0.5;

    public Intake() {
        intakeSpinner = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "IS", Motor.GoBILDA.RPM_1150);
        intakeSpinner.setRunMode(Motor.RunMode.RawPower);
        intakeSpinner.setInverted(true);
        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));
    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (globals.getRobotState() == RobotState.INTAKE) {
                intakeSpinner.set(intakeSpinnerPower);
            } else if (globals.getRobotState() == RobotState.INIT) {
                intakeSpinner.set(intakeSpinnerPower);
            } else {
                intakeSpinner.set(intakeSpinnerPower - 0.3);
            }
            if (Math.abs(intakeSpinner.getVelocity())<25) {intakeSpinner.set(-intakeSpinnerPower);}
//            if (updateAndPowerScheduler.outtakeUpdate) {
//                if (!updateAndPowerScheduler.powerOuttake) {
//                    //disable
//                } else {
//                    //enable
//                }
//                updateAndPowerScheduler.outtakeUpdate = false;
//            }
        }, this);
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Intake Power", intakeSpinner.get());
    }
    public boolean isBusy() {return true;/*return isBusy;*/}
}
