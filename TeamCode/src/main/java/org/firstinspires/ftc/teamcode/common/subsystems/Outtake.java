package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    Servo hoodL;
    Servo hoodR;
    Servo flipper;

    public static double flipUpPos = 0.190;
    public static double flipGroundPos = 0.01;
    public static int flywheelVelocity = 1000;
    public static int turretRTP = 0;
    int adjustedTurretRTP = turretRTP;
    public static double hoodangle = 0;

    public static double P = 0.4;
    public static int ticksPerTurretDegree = 650/90;

    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;

    public Outtake() {
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
        turret.setPositionTolerance(75);
        turret.setPositionCoefficient(P);
        turret.set(0.4);
        turret.stopAndResetEncoder();

        //flywheel.setFeedforwardCoefficients(0.05,0.01,0.31);
        //flywheel.setRunMode(Motor.RunMode.RawPower);

        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }

    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (globals.getRobotState() == RobotState.OUTTAKE) {
                flywheel.setVelocity(flywheelVelocity);
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                hoodL.setPosition(hoodangle);
                hoodR.setPosition(hoodangle);
                if (turret.atTargetPosition()){
                    turret.set(0);
                };
            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(flywheelVelocity*0.75);
                flywheel.setTargetPosition(0);
                hoodL.setPosition(hoodangle);
                hoodR.setPosition(hoodangle);
                flipper.setPosition(flipGroundPos);
                turret.setTargetPosition(turretRTP);
                turret.set(0.4);
                if (turret.atTargetPosition()){
                    turret.set(0);
                };

            } else {
                flywheel.setVelocity(flywheelVelocity*0.75);
                if (OpModeReference.getInstance().limelightSubsystem.angle>3) {
                    adjustedTurretRTP = turretRTP - (int) (ticksPerTurretDegree * OpModeReference.getInstance().limelightSubsystem.angle);
                }
                turret.setTargetPosition(adjustedTurretRTP);
                turret.set(0.4);
            }

//            if (updateAndPowerScheduler.outtakeUpdate) {
//                if (!updateAndPowerScheduler.powerOuttake) {
//                    //disable
//                } else {
//                    //enable
//                }
//                updateAndPowerScheduler.outtakeUpdate=false;
//            }
        }, this);
    }

    public SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            //OpModeReference.getInstance().transfer.transferback().andThen(
            //new WaitCommand(300)),
            new InstantCommand(()->flipper.setPosition(flipUpPos)).andThen(
            new WaitCommand(500)),
            new InstantCommand(()->flipper.setPosition(flipGroundPos)).andThen(
            new WaitCommand(200)).andThen(OpModeReference.getInstance().transfer.transfer().andThen(new WaitCommand(1)))
        );
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Flywheel Velocity", flywheel.getVelocity());
        OpModeReference.getInstance().getTelemetry().addData("Hood Pos", hoodL.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("Flipper Pos", flipper.getPosition());
    }
}
