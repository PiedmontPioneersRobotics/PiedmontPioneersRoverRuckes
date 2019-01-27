package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="tankDrive", group="Iterative Opmode")
public class tankDrive extends LinearOpMode {
    Robot robot = new Robot(hardwareMap);
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            //int position = motor.getCurrentPosition();
            telemetry.addData("Encoder Position", "ghj");

            double sPosition = 0;
            double msPosition = 0;
            double i = -0.01;
            while (i > 0) {

                robot.leftDrive.setPower(gamepad1.left_stick_y / 2);
                robot.rightDrive.setPower(gamepad1.right_stick_y / 2);

                while (gamepad1.b) {
                    robot.servo.setPosition(sPosition);
                    if (sPosition <= 100) {
                        sPosition += 0.005;
                    }
                }
                robot.servo.setPosition(0);

                while (gamepad1.left_bumper) {
                    msPosition += 0.005;
                    robot.Mservo.setPosition(msPosition);
                }

                while (gamepad1.right_bumper) {
                    msPosition += -0.005;
                    robot.Mservo.setPosition(msPosition);
                }

                while (gamepad1.y) {
                    robot.lifter.setPower(0.5);
                }

                while (gamepad1.b) {
                    robot.lifter.setPower(-0.5);
                }
            }
        }
    }
}
