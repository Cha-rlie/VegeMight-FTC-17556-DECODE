package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.OpModeReference;

public class UpdateAndPowerScheduler extends SubsystemBase {
    public boolean powerIntake = false;
    public boolean powerOuttake = false;
    public boolean powerTransfer = false;

    public boolean intakeUpdate = false;
    public boolean outtakeUpdate = false;
    public boolean transferUpdate= false;

    public boolean intakeBusy = false;
    public boolean outtakeBusy = false;
    public boolean transferBusy = false;

    public boolean robotUpdate = false;
    public boolean robotBusy = false;

    Globals globals;

    public UpdateAndPowerScheduler() {
        globals = OpModeReference.getInstance().globalsSubSystem;
    }

    public RunCommand updatePower() {
        return new RunCommand(() -> {
            switch (globals.getRobotState()) {
                case TRAVEL:
                    powerIntake = true;
                    powerOuttake = false;
                    powerTransfer = true;
                case REJECT:
                case INIT:
                case OUTTAKE:
                    powerIntake = true;
                    powerOuttake = true;
                    powerTransfer = true;
                case INTAKE:
                    powerIntake = true;
                    powerOuttake = false;
                    powerTransfer = true;
            }
            if (intakeBusy && outtakeBusy && transferBusy) {
                robotBusy=true;
            } else {
                robotBusy=false;
            }
            if (robotUpdate) {
                intakeUpdate=true;
                outtakeUpdate=true;
                transferUpdate=true;
                robotUpdate=false;
            }
        });
    }

    public InstantCommand intakeUpdate(){
        return new InstantCommand(()->{
           intakeUpdate=true;
        });
    }

    public InstantCommand outtakeUpdate(){
        return new InstantCommand(()->{
           outtakeUpdate=true;
        });
    }

    public InstantCommand transferUpdate(){
        return new InstantCommand(()->{
           transferUpdate = true;
        });
    }

}
