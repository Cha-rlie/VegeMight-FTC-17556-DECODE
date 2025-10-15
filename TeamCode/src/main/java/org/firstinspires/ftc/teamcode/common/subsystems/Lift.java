package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

public class Lift extends SubsystemBase {
    public DcMotorEx motorLiftL;
    public DcMotorEx motorLiftR;

    public int RTP;
    public int adjustment;
    Globals globals;
    public boolean lowBasket;
    private boolean isBusy;

    public Lift() {
        isBusy = false;
        CommandScheduler.getInstance().registerSubsystem(this);
        motorLiftL = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "LS");
        motorLiftR = OpModeReference.getInstance().getHardwareMap().get(DcMotorEx.class, "RS");
        //motorLiftL = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "LS", Motor.GoBILDA.RPM_1150);
        //motorLiftR = new MotorEx(OpModeReference.getInstance().getHardwareMap(), "RS", Motor.GoBILDA.RPM_1150);

        RTP = 0;
        adjustment = 0;
        lowBasket = false;
        OpModeReference.getInstance().getTelemetry().addLine("Slides Initalising");
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorLiftL.stopAndResetEncoder();
        //motorLiftR.stopAndResetEncoder();
        //motorLiftR.setInverted(true);
        motorLiftR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftL.setTargetPositionTolerance(25);
        motorLiftR.setTargetPositionTolerance(25);
        motorLiftL.setTargetPosition(0);
        motorLiftR.setTargetPosition(0);
        motorLiftL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(1);
        motorLiftR.setPower(1);

        globals = OpModeReference.getInstance().globalsSubSystem;

        setDefaultCommand(new PerpetualCommand(goToPosition())); //Perpetual Command is very important, cause InstantCommands end right after they execute
    }

    @Override
    public void periodic() {
        OpModeReference.getInstance().getTelemetry().addData("Lift RTP", RTP);
        OpModeReference.getInstance().getTelemetry().addData("Lift Adjustment", adjustment);
        OpModeReference.getInstance().getTelemetry().addData("Lift Target Position", motorLiftL.getTargetPosition());
        OpModeReference.getInstance().getTelemetry().addData("LS Pos", motorLiftL.getCurrentPosition());
        OpModeReference.getInstance().getTelemetry().addData("LS Power", motorLiftL.getPower());
        OpModeReference.getInstance().getTelemetry().addData("LS Model", motorLiftL.getDeviceName());
        OpModeReference.getInstance().getTelemetry().addData("RS Model", motorLiftR.getDeviceName());
        OpModeReference.getInstance().getTelemetry().addData("RS Pos", motorLiftR.getCurrentPosition());
        OpModeReference.getInstance().getTelemetry().addData("RS Power", motorLiftR.getPower());

        isBusy = true;
    }

    public RunCommand goToPosition() {
        return new RunCommand(() -> {
            isBusy = false;
            if (globals.updateRobotStateTrue && !globals.liftAcceptState) {
                updatePosFromState().schedule();
            }
            motorLiftL.setTargetPosition(RTP + adjustment);
            motorLiftR.setTargetPosition(RTP + adjustment);
            motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLiftL.setTargetPositionTolerance(25);
            motorLiftR.setTargetPositionTolerance(25);
            motorLiftL.setPower(1);
            motorLiftR.setPower(1);
        }, this
        );
    }

    public InstantCommand updatePosFromState() {
        return new InstantCommand(() -> {
//                    globals.liftAcceptState = true;
//                    if (globals.getRobotState() == RobotState.HOVERBEFOREGRAB || globals.getRobotState() == RobotState.GRAB || globals.getRobotState() == RobotState.HOVERAFTERGRAB) {
//                        if (globals.lastRobotState == RobotState.GRAB || globals.lastRobotState == RobotState.HOVERBEFOREGRAB || globals.lastRobotState == RobotState.HOVERAFTERGRAB) {
//                            adjustment = adjustment;
//                        } else {adjustment = 0;}
//                    } else {adjustment = 0;}
//                    switch (globals.getRobotState()) {
//                        case IDLE:
//                            RTP = 0;
//                            break;
//                        case DEPOSIT:
//                        case DEPOSITRELEASE:
//                            if (!lowBasket) {
//                                RTP = 3200;
//                            } else {RTP = 700;}
//                            break;
//                        case HOVERBEFOREGRAB:
//                        case GRAB:
//                        case HOVERAFTERGRAB:
//                            RTP = 615;
//                            break;
//                        default:
//                            RTP = 0;
//                            break;
//                    }
                });
    }

    @NonNull
    public InstantCommand adjustUp(){
        return new InstantCommand(()-> {
                    if (globals.getRobotState() == RobotState.OUTTAKE || globals.getRobotState() == RobotState.IDLE) {
                        if ((RTP+adjustment) + 100 < 3300 /*CHANGE THIS NUMBER*/) {
                            adjustment += 100;
                        } else {adjustment = 3300 - RTP;}
                    } else {
                        if (RTP + adjustment + 100 < 1970) {
                            adjustment += 100;
                        } else {adjustment = 1970 - RTP;}
                    }
                });
    }

    @NonNull
    public InstantCommand adjustDown(){
        return new InstantCommand(()-> {
                    if (RTP + adjustment - 100 > 0) {
                        adjustment -= 100;
                    } else {
                        adjustment = -RTP;
                    }
                });
    }

    public InstantCommand toggleLowBasket() {
        return new InstantCommand(() -> {
           lowBasket = !lowBasket;
        });
    }

    @NonNull
    public InstantCommand resetAdjustment() {
        return new InstantCommand(() -> {
            adjustment = 0;
            OpModeReference.getInstance().getTelemetry().addLine("Lift" +
                    "' Adjustment Reset");
        });
    }

    public boolean isBusy() {
        if (!isBusy) {
            return motorLiftL.isBusy();
        } else return true;
    }

}
