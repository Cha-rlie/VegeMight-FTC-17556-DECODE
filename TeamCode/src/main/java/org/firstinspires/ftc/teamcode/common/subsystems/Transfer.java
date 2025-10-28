package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

public class Transfer extends SubsystemBase {
    CRServo transferR;
    CRServo transferL;
    CRServo transferB;
    UpdateAndPowerScheduler updateAndPowerScheduler;
    Globals globals;


    public Transfer() {
        CommandScheduler.getInstance().registerSubsystem(this);
        globals = OpModeReference.getInstance().globalsSubSystem;
        transferL.setDirection(DcMotorSimple.Direction.REVERSE);
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;

        setDefaultCommand(new PerpetualCommand(defaultCommand()));
    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
           switch(globals.getRobotState()) {
               case INTAKE:
                    transferR.setPower(1);
                    transferL.setPower(1);
               case IDLE:
                   transferR.setPower(0);
                   transferL.setPower(0);
               case OUTTAKE:
                   transferR.setPower(1);
                   transferL.setPower(1);
                   transferB.setPower(1);
           }
        });
    }


}
