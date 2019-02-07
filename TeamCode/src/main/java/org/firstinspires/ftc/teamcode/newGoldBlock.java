package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="findGoldBlock")

public class newGoldBlock extends LinearOpMode {

//usless code
    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.update();

            waitForStart();
            robot.findGoldBlock();


    }
}
