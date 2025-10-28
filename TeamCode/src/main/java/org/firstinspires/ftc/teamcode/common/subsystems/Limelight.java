package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

public class Limelight extends SubsystemBase {
    private Limelight3A limelight;

    double limelightHeight = 17;
    double limelightAngle = 24.920;
    double sampleHeight = 3.5;

    double requiredTotalExtension=250;
    double possibleadjustmentValue =0;
    int storedadjustmentValue=0;

    public Limelight() {
        limelight = OpModeReference.getInstance().getHardwareMap().get(Limelight3A.class, "limelight");
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Required Total LL Extension", requiredTotalExtension);
    }



    public void init(){
        limelight.start();
    }

    public InstantCommand detectObelisk(){
        return new InstantCommand(()-> {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()){
                } else {
                    OpModeReference.getInstance().getTelemetry().addLine("Invalid Result");
                }
            } else {
                OpModeReference.getInstance().getTelemetry().addLine("Null Result");
            }
        });
    }



}
