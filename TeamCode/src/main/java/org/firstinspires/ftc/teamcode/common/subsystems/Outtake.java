package org.firstinspires.ftc.teamcode.common.subsystems;

import android.media.VolumeShaper;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

@Configurable
public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    public MotorEx turret;
    double targetRobotAngle;
    double targetTurretAngle;
    Pose redGoalPose = new Pose(144, 144, 0);
    Pose blueGoalPose = new Pose(0, 144, 0);
    Pose targetGoalPose = redGoalPose;
    Servo hoodL;
    Servo hoodR;
    Servo flipper;
    //1750,0.3

    public static double flipUpPos = 0.190;
    public static double flipGroundPos = 0.01;
    public static int flywheelVelocity = 1000;
    public static int turretRTP = 0;
    int adjustedTurretRTP = turretRTP;
    public static double hoodangle = 0;
    public static double turretAutoCoeffs = 0.05;
    public static double turretPower = 0.1;
    public static double kp = 5;
    public static double kadjust = -1;

    public static double P = 0.05;
    public static int ticksPerTurretDegree = 650/90;

    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {
        targetGoalPose = OpModeReference.getInstance().isRedAlliance ? redGoalPose : blueGoalPose;

        // Set up hardware
        flywheel = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FW", Motor.GoBILDA.BARE);
        turret = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "T", Motor.GoBILDA.RPM_312);
        hoodL = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HL");
        hoodR = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HR");
        hoodL.setDirection(Servo.Direction.REVERSE);
        flipper = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "F");
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        turret.setRunMode(Motor.RunMode.PositionControl);
        flywheel.setInverted(true);
        turret.setPositionTolerance(10);
        turret.setPositionCoefficient(P);
        turret.set(turretPower);
        turret.stopAndResetEncoder();

        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }
    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (!(globals.getRobotState() == RobotState.INIT)) {
                if (!OpModeReference.getInstance().limelightSubsystem.resultIsValid) { // If limelight not seen, then use GoBilda PinPoint
                    turret.set(0);
                }
                else if (Math.abs(OpModeReference.getInstance().limelightSubsystem.angle)>0.01) {
                    turret.setTargetPosition(OpModeReference.getInstance().outtakeSubSystem.turret.getCurrentPosition()-(int)(kp*OpModeReference.getInstance().limelightSubsystem.angle));
                    turret.setPositionCoefficient(turretAutoCoeffs);
                    turret.set(turretPower);
                    if (turret.atTargetPosition()){
                        turret.set(0);
                    };
                } else {
                        turret.setTargetPosition(OpModeReference.getInstance().outtakeSubSystem.turret.getCurrentPosition());
                        turret.set(turretPower);
                        if (turret.atTargetPosition()) {
                            turret.set(0);
                        }
                        ;
                    }

                }


            if (globals.getRobotState() == RobotState.OUTTAKE) {
                flywheel.setVelocity(flywheelVelocity);
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                hoodL.setPosition(hoodangle);
                hoodR.setPosition(hoodangle);

            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(flywheelVelocity*0.75);
                flywheel.setTargetPosition(0);
                hoodL.setPosition(hoodangle);
                hoodR.setPosition(hoodangle);
                flipper.setPosition(flipGroundPos);
                turret.setTargetPosition(turretRTP);
                turret.set(turretPower);
                if (turret.atTargetPosition()){
                    turret.set(0);
                };

            } else {
                flywheel.setVelocity(flywheelVelocity*0.75);
            }
        }, this);
    }

    public InstantCommand alignTurret(){
        return new InstantCommand(()-> {
            targetRobotAngle = Math.atan(Math.toRadians(targetGoalPose.getPose().getY() - OpModeReference.getInstance().pedroPathing.getY()) / (targetGoalPose.getPose().getX() - OpModeReference.getInstance().pedroPathing.getX()));

            targetTurretAngle = targetRobotAngle + OpModeReference.getInstance().pedroPathing.getHeading();

            // Don't let turret turn out of semi-circle bounds
            if (targetTurretAngle <= Math.PI / 2 && targetTurretAngle >= -Math.PI / 2 && turret.atTargetPosition()) {
                turret.setTargetPosition((int) (Math.toDegrees(targetTurretAngle) * ticksPerTurretDegree * kp + turret.getCurrentPosition()));
                turret.setPositionCoefficient(turretAutoCoeffs);
                turret.set(turretPower);
            } else {
                turret.set(0);
            }
            if (turret.atTargetPosition()) {
                turret.set(0);
            }
            ;
        });
    }

    public SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            new InstantCommand(()->flipper.setPosition(flipUpPos)).andThen(
            new WaitCommand(500)),
            new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
            new WaitCommand(200)),
            OpModeReference.getInstance().transfer.transfer().alongWith(
            OpModeReference.getInstance().intakeSubSystem.transfer()),
            new WaitCommand(1)
        );
    }

    public SequentialCommandGroup shootBackTriangle(){
        return new SequentialCommandGroup(
                new InstantCommand(()->flywheelVelocity=1750).andThen(
                new InstantCommand(()->hoodangle=0.2))
        );
    }

    /*public SequentialCommandGroup shootFrontTriangle(){
        return new SequentialCommandGroup(
                new InstantCommand(()->flywheelVelocity= (((int)(1.73975*OpModeReference.getInstance().limelightSubsystem.distance+1130.86772)))).andThen(
                        new InstantCommand(()->hoodangle=0))
        );
    }*/

    public InstantCommand shootFrontTriangle(){
        return new InstantCommand(()-> {
            if (OpModeReference.getInstance().limelightSubsystem.distance > 100) {
                flywheelVelocity = 1530;
            } else {
                flywheelVelocity = 1250;
            }
            hoodangle = 0.1;
        });
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Flywheel Velocity", flywheel.getVelocity());
        OpModeReference.getInstance().getTelemetry().addData("Hood Pos", hoodL.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("Flipper Pos", flipper.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("RobotAngle", targetRobotAngle);
        OpModeReference.getInstance().getTelemetry().addData("TurretAngle", targetTurretAngle);
    }
}
