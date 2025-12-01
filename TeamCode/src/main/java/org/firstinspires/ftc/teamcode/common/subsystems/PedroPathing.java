package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class PedroPathing extends SubsystemBase {
    private Follower follower;

    public PedroPathing() {
        follower = Constants.createFollower(OpModeReference.getInstance().getHardwareMap());
        follower.setPose(OpModeReference.getInstance().getGlobalRobotPose());
        setDefaultCommand(new PerpetualCommand(localise()));
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Heading", follower.getHeading());
        OpModeReference.getInstance().getTelemetry().addData("X", follower.getPose().getX());
        OpModeReference.getInstance().getTelemetry().addData("Y", follower.getPose().getY());
        OpModeReference.getInstance().getTelemetry().addData("Velo X", follower.getVelocity().getXComponent());
        OpModeReference.getInstance().getTelemetry().addData("Velo Y", follower.getVelocity().getYComponent());
    }

    // Default Command
    public RunCommand localise() {
        return new RunCommand(() -> {
            follower.updatePose();
        }, this);
    }

    public InstantCommand resetInHumanPlayerBaseRed() {
        return new InstantCommand(()->follower.setPose(new Pose(136.5, 0, 0)));
    }

    public InstantCommand resetInHumanPlayerBaseBlue() {
        return new InstantCommand(()->follower.setPose(new Pose(7.5, 0, 0)));
    }

    public double getHeading() {
        return follower.getHeading();
    }

    public double getX() {
        return follower.getPose().getX();
    }

    public double getY() {
        return follower.getPose().getY();
    }

    public double getVelocityX() {return follower.getVelocity().getXComponent();}

    public double getVelocityY() {return follower.getVelocity().getYComponent();}

}