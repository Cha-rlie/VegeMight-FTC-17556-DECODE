package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

import java.util.HashMap;

public class Arm extends SubsystemBase {
    Servo leftArm;
    Servo rightArm;

    public static double armPosition = 0.35;
    public static double adjustment = 0;
    private HashMap<RobotState, Double> stateToPositionMap;
    Globals globals;
    private boolean isBusy;

    public Arm() {
        isBusy = false;
        CommandScheduler.getInstance().registerSubsystem(this);
        leftArm = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "LA");
        rightArm = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "RA");
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        stateToPositionMap = new HashMap<RobotState, Double>() {{
            put(RobotState.INIT, 0.48);
            put(RobotState.IDLE, 0.33);
            // ADD WHAT WE NEED HERE
        }};

        globals = OpModeReference.getInstance().globalsSubSystem;

        setDefaultCommand(new PerpetualCommand(turnArm())); //Perpetual Command is very important, cause InstantCommands end right after they execute
    }

    @Override
    public void periodic() {
        //hardware.leftArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        //hardware.rightArm.setPosition(Math.min(armMiniTargetPosition, armTargetPosition));
        OpModeReference.getInstance().getTelemetry().addData("Arm RTP", armPosition);
        OpModeReference.getInstance().getTelemetry().addData("Actual Left Arm Pos", leftArm.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("Actual Right Arm Pos", rightArm.getPosition());
        isBusy = true;
    }

    public RunCommand turnArm() {
        return new RunCommand(() -> {
            isBusy = false;
            if (globals.updateRobotStateTrue) {
                armPosition = stateToPositionMap.get(globals.getRobotState());
                adjustment = 0;
            }
            rightArm.setPosition(armPosition+adjustment);
            leftArm.setPosition(armPosition+adjustment);
        }, this);
    }

    @NonNull
    public InstantCommand adjustArmUp() {
        return new InstantCommand(()-> {
            adjustment = armPosition + adjustment + 0.03 < 1 ? adjustment + 0.03 : 1-armPosition;
            OpModeReference.getInstance().getTelemetry().addLine("Arm Adjusted");
        });
    }

    @NonNull
    public InstantCommand adjustArmDown() {
        return new InstantCommand(()-> {
            adjustment = armPosition + adjustment - 0.03 > 0 ? adjustment - 0.03 : -armPosition;
            adjustment -= 0.03;
            OpModeReference.getInstance().getTelemetry().addLine("Arm Adjusted");
        });
    }

    @NonNull
    public InstantCommand resetAdjustment() {
        return new InstantCommand(() -> {
                    adjustment = 0;
                    OpModeReference.getInstance().getTelemetry().addLine("Arm Adjustment Reset");
                });
    }

    public InstantCommand setArm(double newValue) {
        return new InstantCommand(()-> armPosition = newValue);
    }

    public boolean isBusy() {return isBusy;}

}
