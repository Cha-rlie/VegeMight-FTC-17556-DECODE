/*package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

public class Transfer extends SubsystemBase {
    CRServo transferR;
    CRServo transferL;
    UpdateAndPowerScheduler updateAndPowerScheduler;
    Globals globals;
    
    private double lastPower = 0;
    public boolean override = false;

    public Transfer() {
        CommandScheduler.getInstance().registerSubsystem(this);
        globals = OpModeReference.getInstance().globalsSubSystem;
        transferL = new CRServo(OpModeReference.getInstance().getHardwareMap(), "TL");
        transferR = new CRServo(OpModeReference.getInstance().getHardwareMap(), "TR");
        transferL.setInverted(true);
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;

        setDefaultCommand(new PerpetualCommand(defaultCommand()));
    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
           switch(globals.getRobotState()) {
               case INTAKE:
               case TRANSFER:
                   if (!override) {
                       transferPower(0.8).schedule();
                   }
                    break;
               case OUTTAKE:
                   if (!override) {
                       transferPower(0).schedule();
                   }
                   break;
               default:
                   break;
           }
        }, this);
    }

    public SequentialCommandGroup transfer() {
        return new SequentialCommandGroup(
            new InstantCommand(()->override = true).andThen(),
            transferPower(1),
            new WaitCommand(1000),
            transferPower(0),
            new InstantCommand(()->override = false)
        );
    }
    public SequentialCommandGroup transferBack() {
        return new SequentialCommandGroup(
                new InstantCommand(()->override = true).andThen(),
                transferPower(-0.3),
                new WaitCommand(700),
                transferPower(0),
                new InstantCommand(()->override = false)
        );
    }

    public SequentialCommandGroup transferForwards() {
        return new SequentialCommandGroup(
                new InstantCommand(()->override = true).andThen(),
                transferPower(0.3),
                new WaitCommand(700),
                transferPower(0),
                new InstantCommand(()->override = false)
        );
    }

    public InstantCommand transferPower(double power) {
        return new InstantCommand(()->{
            if (power != lastPower) {
                transferL.set(power);
                transferR.set(power);
                lastPower = power;
            }
        });
    }
}*/
