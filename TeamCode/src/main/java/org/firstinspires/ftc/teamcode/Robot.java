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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

public class Robot {
    /* Public OpMode members. */
    HardwareMap hwMap = null;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo servo;
    public Servo Mservo;
    ModernRoboticsI2cGyro gyro    = null;


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    final double diameter = 4 * 2.54;
    final double pi = Math.PI;
    final double ticksPerInch = 112;
    public void driveForward(double speed, double distance) {
        double encoderDistance = (distance / (diameter * pi)) * ticksPerInch;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftDrive.getCurrentPosition() < encoderDistance &&
                rightDrive.getCurrentPosition() < encoderDistance) {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
        }
    }

    // stuff for turning with gyro
    public double getError(double targetAngle) {

        double robotError;
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    static final double HEADING_THRESHOLD = 1;
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            // Send desired speeds to motors.
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);

        }
        return onTarget;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    static final double P_TURN_COEFF = 0.1;
    // turn with gyro
    public void gyroTurn(double speed, double angle) {
        while (!onHeading(speed, angle, P_TURN_COEFF)) {}
    }
    //turning the servos
    public double MservoMaxDegrees = 135;
    public void moveMegaServo (double angle) {
        Mservo.setPosition((1/MservoMaxDegrees)*angle);
    }
    public double servoMaxDegrees = 90;
    public void moveServo (double angle) {
        servo.setPosition((1/servoMaxDegrees)*angle);
    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftDrive = ahwMap.get(DcMotor.class, "ld");
        rightDrive = ahwMap.get(DcMotor.class, "rd");
        /**
         lifter1 = ahwMap.get(DcMotor.class, "lifter1");
         lifter2 = ahwMap.get(DcMotor.class, "lifter2");
         spinner = ahwMap.get(DcMotor.class, "spinner");
         extender = ahwMap.get(DcMotor.class, "extender");
         s1 = ahwMap.get(Servo.class, "s1");
         s2 = ahwMap.get(Servo.class, "s2");
         ms1 = ahwMap.get(Servo.class, "ms1");
         ms2 = ahwMap.get(Servo.class, "ms2");
         gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
         gyro.calibrate();
         */

        // make sure the gyro is calibrated before continuing
        //while
        /**
         (gyro.isCalibrating())  {}
         */

        // This tells it to run using encoder for the drive motors
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..

        //gyro.resetZAxisIntegrator();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void goToCrater () {}
}