package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

@Configurable
public class Intake extends SubsystemBase {
    MotorEx intakeSpinner;

    Globals globals;

    UpdateAndPowerScheduler updateAndPowerScheduler;
    public static long timerLength = 150;
    ElapsedTime stuckTimer = new ElapsedTime();
    boolean timerRunning = false;

    public static double intakeSpinnerPower = 0.9;
    public boolean override = false;
    boolean allowBackSpin = false;

    public Intake() {
        intakeSpinner = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "IS", Motor.GoBILDA.RPM_312);
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
                intakeSpinner.set(0);
                override = true;
            } else if (globals.getRobotState() == RobotState.OUTTAKE) {
                if (!override) {
                    intakeSpinner.set(0);
                }
            } else {
                intakeSpinner.set(intakeSpinnerPower - 0.15);
            }
            if (Math.abs(intakeSpinner.getVelocity())<25) {
                if (!override && globals.getRobotState() != RobotState.OUTTAKE) {
                    if (timerRunning) {
                        if (stuckTimer.milliseconds() > timerLength) {
                            intakeSpinner.set(-intakeSpinnerPower);
                        }
                    } else {
                        stuckTimer.reset();
                        timerRunning = true;
                    }
                }
                if (allowBackSpin) {
                    if (timerRunning) {
                        if (stuckTimer.milliseconds() > timerLength) {
                            intakeSpinner.set(-intakeSpinnerPower);
                        }
                    } else {
                        stuckTimer.reset();
                        timerRunning = true;
                    }
                }
            } else {
                timerRunning = false;
            }
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

    public SequentialCommandGroup transfer() {
        return new SequentialCommandGroup(
                new InstantCommand(()->override = true),
                new InstantCommand(()->intakeSpinner.set(intakeSpinnerPower)),
                new WaitCommand(1000),
                new InstantCommand(()->intakeSpinner.set(0)),
                new InstantCommand(()->override = false));
    }

    public SequentialCommandGroup intakeBackwards() {
        return new SequentialCommandGroup(
                new InstantCommand(()->override = true).andThen(),
                new InstantCommand(()->intakeSpinner.set(-0.3)),
                new WaitCommand(300),
                new InstantCommand(()->intakeSpinner.set(0.3)),
                new WaitCommand(300),
                new InstantCommand(()->intakeSpinner.set(0)),
                new InstantCommand(()->override = false)
        );
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Intake Power", intakeSpinner.get());
    }
    public boolean isBusy() {return true;/*return isBusy;*/}
}
