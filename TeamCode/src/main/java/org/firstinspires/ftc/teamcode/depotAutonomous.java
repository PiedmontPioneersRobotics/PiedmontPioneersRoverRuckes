package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import static java.lang.Thread.sleep;

@Autonomous(name="depotAutonomous")
public class depotAutonomous extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

            //Lowers down
            robot.lifter.setPower(0.2);
            sleep(1500);
            robot.lifter.setPower(0);

            //lifts the superduperscooper up some
            robot.arm.setPower(0.2);
            sleep(400);
            robot.arm.setPower(0);

            //Drives forward
            robot.driveForward(0.1,135);

            //turns toward crater
            robot.gyroTurn(0.1, 135);

            //drives into crater
            robot.driveForward(0.1, 250);
            //lowers lifter
            robot.lifter.setPower(-0.2);
            sleep(1400);
            robot.lifter.setPower(0);

    }
}
