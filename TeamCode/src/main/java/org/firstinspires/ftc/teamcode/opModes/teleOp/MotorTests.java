package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="MOTORTEST")
@Configurable
public class MotorTests extends OpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    public static double turretLaunchPower = -0.7;
    public static double intakePower = 0.7;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
    }

    @Override
    public void loop() {
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(turretLaunchPower);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setPower(intakePower);
        telemetry.addData("Launch Velocity", motor1.getVelocity());
        telemetry.update();
    }
}
