package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;


public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    Servo hood;
    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {

        flywheel = OpModeReference.getInstance().getHardwareMap().get(MotorEx.class, "Flywheel");

        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setFeedforwardCoefficients(0.05,0.01,0.31);



        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (updateAndPowerScheduler.outtakeUpdate) {
                if (!updateAndPowerScheduler.powerOuttake) {
                    //disable
                } else {
                    //enable
                }
                updateAndPowerScheduler.outtakeUpdate=false;
            }
        });
    }
}
