package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;


public class Outtake extends SubsystemBase {
    public DcMotorEx testing1;
    public DcMotorEx testing2;
    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {

        testing1 = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "testing 1");
        testing2 = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "testing 2");

        testing1.setPower(1);
        testing2.setPower(1);

        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (updateAndPowerScheduler.outtakeUpdate) {
                if (!updateAndPowerScheduler.powerOuttake) {
                    testing1.setMotorDisable();
                    testing2.setMotorDisable();
                } else {
                    testing1.setMotorEnable();
                    testing2.setMotorEnable();
                }
                updateAndPowerScheduler.outtakeUpdate=false;
            }

            testing1.setPower(1);
            testing1.setPower(1);
        });
    }
}
