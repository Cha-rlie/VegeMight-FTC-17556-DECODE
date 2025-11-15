package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

public class DriveTrain extends SubsystemBase {
    public MotorEx FL;
    public MotorEx FR;
    public MotorEx BL;
    public MotorEx BR;
    public MecanumDrive driveTrain;
    private double velocityAdjuster = 1;
    public double adjustment = 0.75;

    Globals globals;
    public DriveTrain() {
        FL = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FL", Motor.GoBILDA.RPM_312);
        FR = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FR", Motor.GoBILDA.RPM_312);
        BL = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "BL", Motor.GoBILDA.RPM_312);
        BR = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "BR", Motor.GoBILDA.RPM_312);
        FL.setInverted(true);
        BL.setInverted(true);

        globals = OpModeReference.getInstance().globalsSubSystem;

        driveTrain = new MecanumDrive(false, FL, FR, BL, BR);
        //setDefaultCommand(new DriveCommand(this));
        //setDefaultCommand(new PerpetualCommand(new InstantCommand(this::driveRobotCentric, this)));
        //setDefaultCommand(new InstantCommand(() -> driveTrain.driveRobotCentric(OpModeReference.getInstance().getGamePad1().getLeftX(), OpModeReference.getInstance().getGamePad1().getLeftY(), OpModeReference.getInstance().getGamePad1().getRightX()), this));
    }

    @Override
    public void periodic() {
        //driveTrain.driveRobotCentric(OpModeReference.getInstance().getGamePad1().getLeftX(), OpModeReference.getInstance().getGamePad1().getLeftY(), OpModeReference.getInstance().getGamePad1().getRightX());
        OpModeReference.getInstance().getTelemetry().addData("Velocity Adjuster", velocityAdjuster);
        if (OpModeReference.getInstance().globalsSubSystem.getRobotState() == RobotState.INTAKE) {
            velocityAdjuster = 0.7;
        } else {
            velocityAdjuster = 1;
        }
        OpModeReference.getInstance().getTelemetry().addData("Robot Driving Automatically", OpModeReference.getInstance().globalsSubSystem.robotAutomaticallyMoving);
        //TODO: print position of bot
        //OpModeReference.getInstance().getTelemetry().addData("Location");
        //OpModeReference.getInstance().getTelemetry().addData("FL Current", FL.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
        //OpModeReference.getInstance().getTelemetry().addData("FL Current", FR.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
        //OpModeReference.getInstance().getTelemetry().addData("FL Current", BL.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
        //OpModeReference.getInstance().getTelemetry().addData("FL Current", FR.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public void driveRobotCentric(double x, double y, double rx) {
        if (globals.getRobotState() == RobotState.DEFENSE){

        }
        driveTrain.driveRobotCentric(x*adjustment, y*adjustment, rx*adjustment, true);
    }

    public InstantCommand decreaseVelocity(double trigger) {
        // TODO: Make this dependent on states later
        return new InstantCommand(()-> velocityAdjuster = trigger > 0.5 ? 0.7 : 1);
    }
}

class DriveCommand extends CommandBase {
    // The subsystem the command runs on
    private final DriveTrain driveTrainSystem;

    public DriveCommand(DriveTrain driveTrainSystem) {
        this.driveTrainSystem = driveTrainSystem;
        addRequirements(this.driveTrainSystem);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {

    }
}
