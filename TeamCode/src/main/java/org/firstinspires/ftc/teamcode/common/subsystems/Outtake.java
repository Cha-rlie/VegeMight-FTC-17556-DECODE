package org.firstinspires.ftc.teamcode.common.subsystems;

import android.media.VolumeShaper;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.CommandScheduler;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Configurable
public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    public MotorEx turret;
    public MotorEx transfer;
    double targetRobotAngle;
    double targetTurretAngle;
    Pose redGoalPose = new Pose(144, 144, 0);
    Pose blueGoalPose = new Pose(0, 144, 0);
    Pose targetGoalPose = redGoalPose;
    Servo hoodL;
    Servo hoodR;
    Servo gate;
    //1750,0.3

    public static int flywheelVelocity = 1000;
    public static double hoodangle = 0;
    public static double turretPower = 0.1;

    public static double P = 0.05;
    public static int ticksPerTurretDegree = 650/90;
    public static int shootFromFarTriangleFlywheelVelo = 1740;
    public static double transferPower = 0.8;
    public static double transferStopPower = 0;
    public static double gatePosOpen = 0;
    public static double gatePosClosed = 0.11;

    public double previousError = 0;
    public double error = 0;
    public double position = 0;
    public double targetPosition = 0;
    public double errorIntegral = 0;
    public double errorDerivative = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public long timePrevious = 0;
    public long timenow = 0;

    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;
    ElapsedTime Timer = new ElapsedTime();

    boolean override = false;

    String mode = "Nothing";
    boolean manualTurretAllowed = true;
    boolean transferOn = false;
    ElapsedTime speedUpTimer = new ElapsedTime();
    boolean timerOn = false;
    public static int speedUpTime = 500;

    public Outtake() {
        targetGoalPose = OpModeReference.getInstance().isRedAlliance ? redGoalPose : blueGoalPose;

        Timer.reset();
        // Set up hardware
        flywheel = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FW", Motor.GoBILDA.BARE);
        turret = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "T", Motor.GoBILDA.RPM_312);
        transfer = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "TW", Motor.GoBILDA.BARE);
        hoodL = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HL");
        hoodR = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HR");
        gate = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "G");
        hoodL.setDirection(Servo.Direction.REVERSE);
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
            if (OpModeReference.getInstance().globalsSubSystem.getRobotState() != RobotState.OUTTAKE) {
                timerOn = false;
            }
            timePrevious=timenow;
            timenow=Timer.nanoseconds();
            //Timer
            turret.setRunMode(Motor.RunMode.RawPower);
            //Calculate P
            previousError=error;
            error=targetPosition-position;
            //Calculate I
            errorIntegral += error*(timePrevious-timenow);
            //Calculate D
            errorDerivative = (previousError-error)/(timePrevious-timenow);

            if (!(globals.getRobotState() == RobotState.INIT) && !override) {
                if (!OpModeReference.getInstance().limelightSubsystem.resultIsValid) { // If limelight not seen, then use GoBilda PinPoint
                    turret.set(0);
                    manualTurretControl(OpModeReference.getInstance().getGamePad2().getRightX());
                }
                else if (Math.abs(OpModeReference.getInstance().limelightSubsystem.angle)>0.01) {

                    turret.set(error*kP+errorIntegral*kI+errorDerivative*kD);

                } else {
                        manualTurretControl(OpModeReference.getInstance().getGamePad2().getRightX());
                        turret.setTargetPosition(OpModeReference.getInstance().outtakeSubSystem.turret.getCurrentPosition());
                        turret.set(turretPower);
                        if (turret.atTargetPosition()) {
                            turret.set(0);
                        }
                    }

                }


            if (globals.getRobotState() == RobotState.OUTTAKE) {
                flywheel.setVelocity(flywheelVelocity);
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                hoodL.setPosition(hoodangle);
                hoodR.setPosition(hoodangle);
                if (!timerOn) {
                    timerOn = !timerOn;
                    speedUpTimer.reset();
                }
                if (speedUpTimer.time(TimeUnit.MILLISECONDS) > speedUpTime) {
                    gate.setPosition(gatePosOpen);
                    transfer.set(transferPower);
                }

            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(0);
                turret.set(0);
                transfer.set(transferStopPower);
            } else if (globals.getRobotState()==RobotState.DEFENSE){
                flywheel.setVelocity(0);
                transfer.set(transferStopPower);
            } else {
                flywheel.setVelocity(flywheelVelocity*0.85);
                transfer.set(transferStopPower);
                gate.setPosition(gatePosClosed);
            }
        }, this);
    }

    public SequentialCommandGroup toggleContinuousShoot() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> {
                    transferOn = !transferOn;
                    if (transferOn) {
                        transfer.set(transferPower);
                    } else {
                        transfer.set(transferStopPower);
                    }
                })
        );
    }

    public SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
            new WaitCommand(400),
            //OpModeReference.getInstance().transfer.transferBack().alongWith(new InstantCommand(()->flipper.setPosition(flipUpPos))),
            new WaitCommand(500),
            //new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
            new WaitCommand(200),
            //OpModeReference.getInstance().transfer.transfer().alongWith(
            OpModeReference.getInstance().intakeSubSystem.transfer(),
            new WaitCommand(1)
        );
    }

    public SequentialCommandGroup stuckShoot() {
        return new SequentialCommandGroup(
                OpModeReference.getInstance().globalsSubSystem.setRobotStateCommand(RobotState.OUTTAKE),
                new WaitCommand(400),
                //OpModeReference.getInstance().transfer.transferBack().alongWith(OpModeReference.getInstance().intakeSubSystem.intakeBackwards()).alongWith(new InstantCommand(()->flipper.setPosition(flipUpPos))),
                new WaitCommand(500),
                //new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
                        new WaitCommand(200),
                //OpModeReference.getInstance().transfer.transfer().alongWith(
                        OpModeReference.getInstance().intakeSubSystem.transfer(),
                new WaitCommand(1)
        );
    }

    public SequentialCommandGroup autonomousShoot() {
        return new SequentialCommandGroup(
                //new InstantCommand(()->OpModeReference.getInstance().transfer.transferBack()),
                //new InstantCommand(()->flipper.setPosition(flipUpPos)).andThen(
                        new WaitCommand(500),
                //new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
                        new WaitCommand(200),
                //OpModeReference.getInstance().transfer.transfer().alongWith(
                        OpModeReference.getInstance().intakeSubSystem.transfer(),
                new WaitCommand(1)
        );
    }

    public SequentialCommandGroup shootBackTriangle(){
        return new SequentialCommandGroup(
                new InstantCommand(()->mode="Back Triangle"),
                new InstantCommand(()->flywheelVelocity=shootFromFarTriangleFlywheelVelo).andThen(
                new InstantCommand(()->hoodangle=0.2))
        );
    }

    public InstantCommand shootFrontTriangle(){
        return new InstantCommand(()-> {
            mode = "Front Triangle";
            if (OpModeReference.getInstance().limelightSubsystem.distance > 100) {
                flywheelVelocity = 1530;
            } else {
                flywheelVelocity = 1250;
            }
            hoodangle = 0.1;
        });
    }

    public SequentialCommandGroup turretTurnToPos(int targetPos) {
        return new SequentialCommandGroup(
            new InstantCommand(()-> override = true),
            new InstantCommand(()->turret.setTargetPosition(targetPos)),
            new InstantCommand(()->turret.set(turretPower)),
            new WaitUntilCommand(()->turret.atTargetPosition()).withTimeout(800),
            new InstantCommand(()->turret.set(0)),
            new InstantCommand(()-> override = false));
    }

    public void manualTurretControl(double joyStickValue) {
        turret.setTargetPosition(turret.getCurrentPosition()+(int)joyStickValue*30);
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Flywheel Velocity", flywheel.getVelocity());
        OpModeReference.getInstance().getTelemetry().addData("Hood Pos", hoodL.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("RobotAngle", targetRobotAngle);
        OpModeReference.getInstance().getTelemetry().addData("TurretAngle", targetTurretAngle);
        OpModeReference.getInstance().getTelemetry().addData("Mode", mode);
    }
}
