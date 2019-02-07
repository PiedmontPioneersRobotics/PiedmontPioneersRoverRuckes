package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="tankDrive", group="Iterative Opmode")
public class tankDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(hardwareMap);

//        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            //int position = motor.getCurrentPosition();
            telemetry.addData("Encoder Position", "ghj");

            waitForStart();

            while (0 == 0) {
                robot.leftDrive.setPower(gamepad1.left_stick_y);
                robot.rightDrive.setPower(gamepad1.right_stick_y);

                double sPosition = 0;
                robot.servo.setPosition(sPosition);
                while (gamepad1.b) {
                    robot.servo.setPosition(sPosition);
                    if (sPosition <= 100) {
                        sPosition += 0.005;
                    }
                }

                while (gamepad1.left_bumper) {
                    robot.arm.setPower(0.2);
                }
                robot.arm.setPower(0);

                while (gamepad1.right_bumper) {
                    robot.arm.setPower(-0.2);
                }
                robot.arm.setPower(0);

                if (gamepad1.y) {
                    robot.lifter.setPower(0.2);
                    sleep(1500);
                    robot.lifter.setPower(0);
                }

                if (gamepad1.a) {
                    robot.lifter.setPower(-0.2);
                    sleep(1500);
                    robot.lifter.setPower(0);
                }

            }
//        }
    }
}