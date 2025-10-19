package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.OpModeReference;
import org.firstinspires.ftc.teamcode.common.util.Globals;
import org.firstinspires.ftc.teamcode.common.util.RobotState;

import java.util.HashMap;

public class Intake extends SubsystemBase {
    DcMotorEx intakeSpinner;


    Globals globals;
    private boolean isBusy;

    public Intake() {
        isBusy = false;
        CommandScheduler.getInstance().registerSubsystem(this);
        globals = OpModeReference.getInstance().globalsSubSystem;
    }

    @Override
    public void periodic() {
        isBusy = true;
    }
    public boolean isBusy() {return isBusy;}
}
