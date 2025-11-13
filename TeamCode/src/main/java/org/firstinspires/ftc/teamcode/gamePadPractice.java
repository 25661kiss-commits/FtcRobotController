package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class gamePadPractice extends OpMode {


        @Override
         public void init() {



        }

        @Override
    public void loop() {
            double value = gamepad1.left_stick_y-gamepad1.right_stick_y;
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("Ly",gamepad1.left_stick_y);

            telemetry.addData("ry",gamepad1.right_stick_y);
            telemetry.addData("A",gamepad1.a);
            telemetry.addData("diff",value);
        }
}

