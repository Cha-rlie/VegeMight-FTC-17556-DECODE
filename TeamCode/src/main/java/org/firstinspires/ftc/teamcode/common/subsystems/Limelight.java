package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.OpModeReference;

@Configurable
public class Limelight extends SubsystemBase {
    private Limelight3A limelight;

    public static double limelightHeight = 40;
    public static double limelightAngle = 15;
    public static int txMultiplier = 1;
    double aprilTagHeight = 74.95;
    double angle = 0;

    double distance = 100;

    public boolean resultIsValid = true;

    public Limelight() {
        limelight = OpModeReference.getInstance().getHardwareMap().get(Limelight3A.class, "limelight");
        OpModeReference.getInstance().getTelemetry().setMsTransmissionInterval(11);
        limelight.pipelineSwitch(OpModeReference.getInstance().isRedAlliance ? 0 : 1);
        limelight.start();
        setDefaultCommand(new PerpetualCommand(calibration()));
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Distance to AprilTag", distance);
        OpModeReference.getInstance().getTelemetry().addData("Angle to AprilTag", angle);
        OpModeReference.getInstance().getTelemetry().addData("Ty", limelight.getLatestResult().getTy());
        OpModeReference.getInstance().getTelemetry().addData("Tx", limelight.getLatestResult().getTx());
    }

    // Default Command
    public RunCommand calibration(){
        return new RunCommand(()-> {
            if (limelight.getLatestResult().isValid()) {
                resultIsValid = true;
                double targetOffsetAngle_Vertical = - limelight.getLatestResult().getTy();

                // how many degrees back is your limelight rotated from perfectly vertical;

                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = limelightHeight;

                // distance from the target to the floor
                double goalHeightInches = aprilTagHeight;

                double angleToGoalDegrees = limelightAngle + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                //calculate distance
                distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                angle = limelight.getLatestResult().getTx();
            } else if (!limelight.getLatestResult().isValid()) {
                resultIsValid=false;
            }
            }, this);
    }

    public InstantCommand detectObelisk() {
        return new InstantCommand();
    }

    public InstantCommand resetLimeLight() {
        return new InstantCommand(()-> {

        });
    }

}
