package org.firstinspires.ftc.teamcode.common.subsystems;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;
import org.firstinspires.ftc.teamcode.common.util.UpdateAndPowerScheduler;

@Configurable
public class Outtake extends SubsystemBase {
    public MotorEx flywheel;
    public MotorEx turret;
    public MotorEx transfer;
    double targetRobotAngle;
    double targetTurretAngle;
    int turretTargetPos;
    Pose redGoalPose = new Pose(144, 144, 0);
    Pose blueGoalPose = new Pose(0, 144, 0);
    Pose targetGoalPose = redGoalPose;
    Servo hoodL;
    Servo hoodR;
    Servo gate;
    //1750,0.3

    public static int flywheelVelocity = 1000;
    public static double hoodangle = 0;
    public static double hoodDownAngle = 0.22;
    public static double turretPower = 0.1;

    public static double P = 0.05;
    public static int ticksPerTurretDegree = 650/90;
    public static int shootFromFarTriangleFlywheelVelo = 1740;
    public static double transferPower = 0.8;
    public static double transferStopPower = 0;
    public static double gatePosOpen = 0.3;
    public static double gatePosClosed = 0.17;

    public double previousError = 0;
    public double error = 0;
    public double position = 0;
    public double targetPosition = 0;
    public double errorIntegral = 0;
    public double errorDerivative = 0;
    public static double kP = 0.006;
    public double kI = 0;
    public double kD = 0;

    public long timePrevious = 0;
    public long timenow = 0;

    Globals globals;
    UpdateAndPowerScheduler updateAndPowerScheduler;
    ElapsedTime Timer = new ElapsedTime();

    boolean override = false;

    String mode = "Back Triangle";
    boolean manualTurretAllowed = true;
    boolean transferOn = false;
    boolean allowShoot = false;
    ElapsedTime angleTimer = new ElapsedTime();
    boolean angleAdjusted = false;
    boolean timerOn = false;
    public static int angleTime = 250;
    int[] lastVelocity = new int[8];

    public Outtake() {
        targetGoalPose = OpModeReference.getInstance().isRedAlliance ? redGoalPose : blueGoalPose;

        Timer.reset();
        // Set up hardware
        flywheel = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "FW", Motor.GoBILDA.BARE);
        turret = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "T", Motor.GoBILDA.RPM_312);
        turret.resetEncoder();
        transfer = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "TW", Motor.GoBILDA.BARE);
        transfer.setInverted(true);
        hoodL = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HL");
        hoodR = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "HR");
        gate = OpModeReference.getInstance().getHardwareMap().get(Servo.class, "G");
        hoodL.setDirection(Servo.Direction.REVERSE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setPositionTolerance(25);
        flywheel.setInverted(true);
        turret.setPositionTolerance(10);
        turret.setPositionCoefficient(P);
        turret.set(turretPower);
        turret.stopAndResetEncoder();


        globals = OpModeReference.getInstance().globalsSubSystem;
        updateAndPowerScheduler = OpModeReference.getInstance().updateAndPowerScheduler;
        shootBackTriangle().schedule();
        setDefaultCommand(new PerpetualCommand(defaultCommand()));

    }
    public RunCommand defaultCommand() {
        return new RunCommand(()->{
            if (OpModeReference.getInstance().globalsSubSystem.getRobotState() != RobotState.OUTTAKE) {
                timerOn = false;
            }
            if (allowShoot) {
                gate.setPosition(gatePosOpen);
                transfer.set(transferPower);
            } else {
                gate.setPosition(gatePosClosed);
                transfer.set(transferStopPower);
            }
            timePrevious=timenow;
            timenow=Timer.nanoseconds();
            //Timer
            turret.setRunMode(Motor.RunMode.RawPower);
            //Calculate P
            previousError=error;
            error=(turretTargetPos-turret.getCurrentPosition());
            //Calculate I
            errorIntegral += error*(timenow-timePrevious);
            //Calculate D
            errorDerivative = (previousError-error)/(timenow-timePrevious);

            // Turret stuff
            if (globals.getRobotState() != RobotState.INIT && !override) {
                double robotAngle = Math.toDegrees(OpModeReference.getInstance().pedroPathing.getHeading());
                if (robotAngle < -10) {
                    robotAngle = robotAngle+180;
                    double requiredAngle = OpModeReference.getInstance().kalmanfilter.ReqAngle;
                    if ((int) (robotAngle- requiredAngle)<-135) {
                        targetTurretAngle = -135;
                    } else if ((int) (robotAngle- requiredAngle)>135) {
                        targetTurretAngle = 135;
                    } else {
                        targetTurretAngle = (int) (robotAngle- requiredAngle);
                    }
                } else if (robotAngle > 10){
                    robotAngle = robotAngle-180;
                    double requiredAngle = OpModeReference.getInstance().kalmanfilter.ReqAngle;
                    if ((int) (robotAngle- requiredAngle)<-135) {
                        targetTurretAngle = -135;
                    } else if ((int) (robotAngle- requiredAngle)>135) {
                        targetTurretAngle = 135;
                    } else {
                        targetTurretAngle = (int) (robotAngle- requiredAngle);
                    }
                }




                // Take Kalman Filter angle (pi to -pi)

                // Find Robot Angle -> (pi to -pi) given
                // Convert to Turret angle -> (pi to -pi) 180+180 -> if RA > 0; angle = RA-180; else +180


                // Spin to the angle given
                turretTargetPos = (int) (ticksPerTurretDegree*targetTurretAngle);
                turret.setTargetPosition(turretTargetPos);
                turret.set(error*kP+errorIntegral*kI+errorDerivative*kD);
                /*if ((turretTargetPos - turret.getCurrentPosition() < 0) && !turret.atTargetPosition()) {
                    turretPower = turretPower * -1;
                }
                turret.set(turretPower);*/
                if (turret.atTargetPosition()) {
                    turret.set(0);
                    if (OpModeReference.getInstance().limelightSubsystem.resultIsValid) {

                    }
                }
            }

//            if (!(globals.getRobotState() == RobotState.INIT) && !override) {
//                if (OpModeReference.getInstance().limelightSubsystem.resultIsValid) {
//                    turret.set(0);
//                    manualTurretControl(OpModeReference.getInstance().getGamePad2().getRightX());
//                }
//                else if (Math.abs(OpModeReference.getInstance().limelightSubsystem.angle)>0.01) {
//                    turret.set(error*kP+errorIntegral*kI+errorDerivative*kD);
//                } else {
//                        manualTurretControl(OpModeReference.getInstance().getGamePad2().getRightX());
//                        turret.setTargetPosition(OpModeReference.getInstance().outtakeSubSystem.turret.getCurrentPosition());
//                        turret.set(turretPower);
//                        if (turret.atTargetPosition()) {
//                            turret.set(0);
//                        }
//                    }
//
//                }


            if (globals.getRobotState() == RobotState.OUTTAKE) {
                flywheel.setVelocity(flywheelVelocity);
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
                if (mode.equals("Front Triangle")) {
                    flywheelVelocity = (int) ((3.32105*(OpModeReference.getInstance().kalmanfilter.ReqDist))+1203.52797);
                    //// could use lowest setting; trajectory cannot reach at lower distances then 100 i.e;
                    //regression

                    if (OpModeReference.getInstance().kalmanfilter.ReqDist<125){
                        flywheelVelocity = 1600;
                        hoodangle = 0.22;
                    }

                    if (allowShoot && angleTimer.milliseconds() > angleTime) {
                        hoodangle = 0.22;
                        angleTimer.reset();
                        angleAdjusted = true;
                    } else if (angleAdjusted && angleTimer.milliseconds() > 500) {
                        hoodangle = 0.43;
                        angleAdjusted = false;
                    } else if (!angleAdjusted) {
                        hoodangle = 0.43;
                    }
                    for (int i = lastVelocity.length - 1; i > 0; i--) {
                        lastVelocity[i] = lastVelocity[i-1];
                    }
                    lastVelocity[0] = (int) flywheel.getVelocity();
                    hoodL.setPosition(hoodangle);
                    hoodR.setPosition(hoodangle);
                }
                if (mode.equals("Back Triangle")) {
                    flywheelVelocity = 1950;
                    if (allowShoot && angleTimer.milliseconds() > angleTime) {
                        hoodangle = 0.22;
                        angleTimer.reset();
                        angleAdjusted = true;
                    } else if (angleAdjusted && angleTimer.milliseconds() > 500) {
                        hoodangle = 0.4;
                        angleAdjusted = false;
                    } else if (!angleAdjusted) {
                        hoodangle = 0.4;
                    }
                    for (int i = lastVelocity.length - 1; i > 0; i--) {
                        lastVelocity[i] = lastVelocity[i-1];
                    }
                    lastVelocity[0] = (int) flywheel.getVelocity();
                    hoodL.setPosition(hoodangle);
                    hoodR.setPosition(hoodangle);
                }
                /*if (!timerOn) {
                    timerOn = !timerOn;
                    speedUpTimer.reset();
                }
                if (speedUpTimer.time(TimeUnit.MILLISECONDS) > speedUpTime) {
                    gate.setPosition(gatePosOpen);
                    transfer.set(transferPower);
                }*/

            } else if (globals.getRobotState() == RobotState.INIT) {
                flywheel.setVelocity(0);
                turret.set(0);
                transfer.set(transferStopPower);
            } else if (globals.getRobotState() == RobotState.INTAKE || globals.getRobotState() == RobotState.TRAVEL) {
                    transfer.set(transferStopPower);
                    gate.setPosition(gatePosClosed);
                    flywheel.setVelocity(flywheelVelocity*0.85);
                    allowShoot = false;
            } else if (globals.getRobotState()==RobotState.DEFENSE){
                flywheel.setVelocity(0);
                transfer.set(transferStopPower);
                gate.setPosition(gatePosClosed);
                allowShoot = false;
            } else {
                flywheel.setVelocity(flywheelVelocity*0.85);
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

    public InstantCommand shoot() {
        return new InstantCommand(()-> {
            allowShoot = !allowShoot;
            angleTimer.reset();
        });
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
                new InstantCommand(()->flywheelVelocity=shootFromFarTriangleFlywheelVelo));
    }

    public SequentialCommandGroup shootFrontTriangle(){
        return new SequentialCommandGroup(
                new InstantCommand(()->mode="Front Triangle"),
                new InstantCommand(()->flywheelVelocity=shootFromFarTriangleFlywheelVelo));
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
        OpModeReference.getInstance().getTelemetry().addData("Last Velocities", lastVelocity);
        OpModeReference.getInstance().getTelemetry().addData("Hood Pos", hoodL.getPosition());
        OpModeReference.getInstance().getTelemetry().addData("TurretAngle", targetTurretAngle);
        OpModeReference.getInstance().getTelemetry().addData("Current Turret Pos", turret.getCurrentPosition());
        OpModeReference.getInstance().getTelemetry().addData("Target Turret Pos", turretTargetPos);
        OpModeReference.getInstance().getTelemetry().addData("Mode", mode);
    }
}
