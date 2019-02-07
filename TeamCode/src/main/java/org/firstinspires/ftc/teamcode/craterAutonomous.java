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

@Autonomous(name="craterAutonomous")

public class craterAutonomous extends LinearOpMode {


    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.update();

            waitForStart();

            //Lowers down
            robot.lifter.setPower(0.2);
            sleep(1500);
            robot.lifter.setPower(0);

            //lifts the superduperscooper up a bit
            robot.arm.setPower(0.3);
            sleep(1000);
            robot.arm.setPower(0);


            //Drives forward
            robot.driveForward(0.3,135);
            //lowers lifter
            robot.lifter.setPower(-0.2);
            sleep(1400);
            robot.lifter.setPower(0);
    }
}
