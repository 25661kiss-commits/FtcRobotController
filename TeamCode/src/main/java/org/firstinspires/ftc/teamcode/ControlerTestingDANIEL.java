package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ControlerTestingDANIEL extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double speedForward = -gamepad1.left_stick_y / 2;
        telemetry.addData("right stick X:", gamepad1.right_stick_x);
        telemetry.addData("right stick Y:", gamepad1.right_stick_y);
        telemetry.addData("left stick X:", gamepad1.left_stick_x);
        telemetry.addData("left stick Y:", speedForward);
        telemetry.addData("stickdiff:", gamepad1.left_stick_x - gamepad1.right_stick_x);
        telemetry.addData("triggersum:", gamepad1.left_trigger + gamepad1.right_trigger);
        telemetry.addData("button A/1:", gamepad1.a);
        telemetry.addData("button B/2:", gamepad1.b);
    }
}
