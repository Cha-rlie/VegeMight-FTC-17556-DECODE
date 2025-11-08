package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
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

    public Transfer() {
        CommandScheduler.getInstance().registerSubsystem(this);
        globals = OpModeReference.getInstance().globalsSubSystem;
        transferL = new CRServo(OpModeReference.getInstance().getHardwareMap(), "TL");
        transferR = new CRServo(OpModeReference.getInstance().getHardwareMap(), "TR");
        transferL.setInverted(true);
        //transferL.set(0);
        //transferR.set(0);
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;

        setDefaultCommand(new PerpetualCommand(defaultCommand()));
    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
           switch(globals.getRobotState()) {
               case INTAKE:
               case INIT:
                    transfer(1);
               case TRANSFER:
                   transfer(1);
               case OUTTAKE:
                   transfer(0);
           }
        }, this);
    }

    public RunCommand transfer() {
        return new RunCommand(()->{
            transfer(1);
            new WaitCommand(2000);
            transfer(0);
        });
    }
    
    public void transfer(double power) {
        if (power != lastPower) {
            transferL.set(power);
            transferR.set(power);
            lastPower = power;
        }
    }


}
