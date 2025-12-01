package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

public class Flag extends SubsystemBase {
    Servo flag;
    Globals globals;
    boolean flagActived = false;
    public static double flagZeroPos = 0.05;

    public Flag() {
        CommandScheduler.getInstance().registerSubsystem(this);
        flag = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "F");
        flag.setDirection(Servo.Direction.REVERSE);

        globals = OpModeReference.getInstance().globalsSubSystem;

        setDefaultCommand(new PerpetualCommand(whenActiveatedFlagExtendo()));
    }

    @Override
    public void periodic() {
        //hardware.leftArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        //hardware.rightArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        OpModeReference.getInstance().getTelemetry().addData("Flag State", flagActived);
        OpModeReference.getInstance().getTelemetry().addData("Flag Pos", flag.getPosition());
    }

    public RunCommand whenActiveatedFlagExtendo() {
        return new RunCommand(() -> {
           if (flagActived) {
               flag.setPosition(0.375);
           } else {
               flag.setPosition(0);
           }
        }, this);
    }


    public RunCommand whenParkFlagExtendo() {
        return new RunCommand(() -> {
            if(globals.getRobotState() == RobotState.PARK) {
                flag.setPosition(flagZeroPos);
            } else {
                flag.setPosition(0.55);
            }
        }, this);
    }

    public InstantCommand toggleFlag() {
        return new InstantCommand(() -> {
            flagActived = !flagActived;
        });
    }
}
