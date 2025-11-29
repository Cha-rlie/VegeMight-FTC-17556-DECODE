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

@Configurable
public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    public MotorEx turret;
    public MotorEx transferSecondary;
    double targetRobotAngle;
    double targetTurretAngle;
    Pose redGoalPose = new Pose(144, 144, 0);
    Pose blueGoalPose = new Pose(0, 144, 0);
    Pose targetGoalPose = redGoalPose;
    Servo hoodL;
    Servo hoodR;
    //1750,0.3

    public static int flywheelVelocity = 1000;
    public static double hoodangle = 0;
    public static double turretPower = 0.1;

    public static double P = 0.05;
    public static int ticksPerTurretDegree = 650/90;
    public static int shootFromFarTriangleFlywheelVelo = 1740;

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

    public Outtake() {
        targetGoalPose = OpModeReference.getInstance().isRedAlliance ? redGoalPose : blueGoalPose;

        Timer.reset();
        // Set up hardware
        flywheel = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FW", Motor.GoBILDA.BARE);
        turret = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "T", Motor.GoBILDA.RPM_312);
        transferSecondary = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "Transfer", Motor.GoBILDA.RPM_1150);
        hoodL = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HL");
        hoodR = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HR");
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

            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(0);
                turret.set(0);
            } else if (globals.getRobotState()==RobotState.DEFENSE){
                flywheel.setVelocity(0);
            } else {
                flywheel.setVelocity(flywheelVelocity*0.75);
            }
        }, this);
    }

    public InstantCommand shoot() {
        return new InstantCommand(()-> {
            transferSecondary.set(1);
        });
    }

    public SequentialCommandGroup autonomousShoot() {
        return new SequentialCommandGroup(

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
