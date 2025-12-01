package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="MOTORTEST")
@Configurable
public class MotorTests extends OpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    Servo angleL;
    Servo angleR;
    //Servo flipper;
    Servo transferL;
    Servo transferR;
    public static double turretLaunchPower = -0.7;
    public static double intakePower = 0.7;
    public static double anglePos = 0;
    public static double flipperPos = 0;
    public static double transferPower = 0;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        angleL = hardwareMap.get(Servo.class, "angleL");
        angleR = hardwareMap.get(Servo.class, "angleR");
        angleR.setDirection(Servo.Direction.REVERSE);
        //flipper = hardwareMap.get(Servo.class, "flipper");
    }

    @Override
    public void loop() {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(turretLaunchPower);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setPower(intakePower);
        angleL.setPosition(anglePos);
        angleR.setPosition(anglePos);
        //flipper.setPosition(flipperPos);
        telemetry.addData("Launch Velocity", motor1.getVelocity());
        telemetry.update();
    }
}
