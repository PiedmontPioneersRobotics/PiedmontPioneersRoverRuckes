/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tank drive", group="Iterative Opmode")
//@Disabled
public class tankDrive extends OpMode
{
    /**
     * this starts all of the motors and servos and gyro
     */

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo servo;
    public Servo Mservo;

    ModernRoboticsI2cGyro gyro    = null;

    // Declare OpMode members.
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

@Override
    public void init() {


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    /**
     * this defines all of the motors/servos/gyros to the phone
     */
    leftDrive = hardwareMap.get(DcMotor.class, "ld");
    rightDrive = hardwareMap.get(DcMotor.class, "rd");
    servo = hardwareMap.get(Servo.class, "s1");
    Mservo = hardwareMap.get(Servo.class, "ms1");

    leftDrive.setDirection(DcMotor.Direction.REVERSE);
    gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
    gyro.calibrate();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double power;
        //int position = motor.getCurrentPosition();
        telemetry.addData("Encoder Position", "ghj");
        power = gamepad1.left_stick_y;
        leftDrive.setPower(power);
        power = gamepad1.right_stick_y;
        rightDrive.setPower(power);

        double msPosition;
        while (gamepad1.b) {
            double sPosition = 0;
            servo.setPosition(sPosition);
            if (sPosition <= 100) {
                sPosition += 0.01;
            }
        }
        servo.setPosition(0);

        while (gamepad1.left_bumper) {
            msPosition += 0.01
        }



        // robot.init(hardwareMap);
        // Setup a variable for each drive wheel to save power level for telemetry
        /**
         * this is how the joysticks drive the wheels
          */
        /**
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //this could make it slow
        double rightX = gamepad1.right_stick_x;
        */



        /**
         * It tells what the keys on the gamepad are meant to do
         */
        /**
        if (gamepad1.b) {
            spinner.setPower(.5);
        } else {
            spinner.setPower(0);
        }
        if (gamepad1.dpad_up) {
            extender.setPower(1);
        } else {
            extender.setPower(0);
        }
        if (gamepad1.dpad_down) {
            extender.setPower(-1);
        } else {
            extender.setPower(0);
        }
        if (gamepad1.y) {
            lifter1.setPower(1);
            lifter2.setPower(1);
        } else {
            lifter1.setPower(0);
            lifter2.setPower(0);
        }
        if (gamepad1.a) {
            lifter1.setPower(-1);
            lifter2.setPower(-1);
        } else {
            lifter1.setPower(0);
            lifter2.setPower(0);
        }
        if (gamepad1.left_trigger > 0) {
            ms1.setPosition(gamepad1.left_trigger);
            ms2.setPosition(gamepad1.left_trigger);
        } else {
            ms1.setPosition(0);
            ms2.setPosition(0);
        }
        if (gamepad1.right_trigger > 0) {
            s1.setPosition(gamepad1.right_trigger);
            s2.setPosition(gamepad1.right_trigger);
        } else {
            s1.setPosition(0);
            s2.setPosition(0);
        }
        */


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", leftDrive);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override

    /**
     * this stops all the motors
     */

    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

 /*

        ADD LEFT DRIVE AND RIGHT DRIVE.

    public void encoderDrive(double speed,

                              double leftInches, double rightInches,
                              double timeoutS) {
    int newLeftTarget;
    int newRightTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }
} */
}
