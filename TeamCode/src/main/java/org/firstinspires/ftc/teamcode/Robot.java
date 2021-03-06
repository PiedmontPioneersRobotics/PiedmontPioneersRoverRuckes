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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    HardwareMap hwMap;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor lifter;
    public DcMotor arm;
    public Servo servo;


    ModernRoboticsI2cGyro gyro;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AfZ82G//////AAABmTCz0kBQDEwzrKIhsIwJFIVm20yDWkFHq2aAgA/k95YeAe2INqqkc9ZLSIPRIu3PF1ojmeVgb4VUI7J6Qays7ISspBJYdklKOdghZj6ucwy/ii7FqhaQAuqxXXXYJQ+/Ixx5HVbIDkhjUWZo76WGfiG1UGDogXGG+GDx68nRog6Zd09g5tYLqITCAmUSQf46n6KRgpJOBnf/a8nERAlwqeP+3Mp+nUL8QIy2aTIxmU7HSk15ocB0o40OQx4cAmlJvPfhF+kV+tckkM0EMcBuQa4MUPbPLA77LgZ0pzPaaH9fYNAQJCuwvcLtRD0Jr1Ze/cSXi9ITaZGanGdJoeTZ4hCeVA2dV4lJaui9hd/PtWid\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    final double diameter = 10.29;
    final double pi = Math.PI;
    final double ticksPerRotation = 280;
    public Robot() {}
    public void driveForward(double speed, double distance) {
        double encoderDistance = (distance / (diameter * pi)) * ticksPerRotation;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (leftDrive.getCurrentPosition() < encoderDistance &&
                rightDrive.getCurrentPosition() < encoderDistance) {
            leftDrive.setPower(-speed);
            rightDrive.setPower(-speed);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

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



    public double servoMaxDegrees = 90;
    public void moveServo (double angle) {
        servo.setPosition((1/servoMaxDegrees)*angle);
    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        gyro = (ModernRoboticsI2cGyro)ahwMap.gyroSensor.get("gyro");
        gyro.calibrate();
        leftDrive = (DcMotor)ahwMap.get("ld");
        rightDrive = (DcMotor)ahwMap.get("rd");
        lifter = (DcMotor)ahwMap.get("lifter");
        arm = (DcMotor)ahwMap.get("arm");

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void findGoldBlock () {

    }

}