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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous", group="Linear Opmode")
//@Disabled
public class AutonomousTest extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
//        goToCrater(1);
//        goToCrater(2);
    //    findGold();
    //    deployMarker();
        // run until the end of the match (driver presses STOP)

    }
    /**
     * Deploys marker from position POSN.
     * POSN = 1 is starting spot in front of crater.
     * POSN = 2 is starting spot in front of depot.
     */
    public void deployMarker(int posn){
        if(posn == 1){
            robot.driveForward(0.1, 10);
            robot.gyroTurn(0.1, 270);
            robot.driveForward(0.1, 130);
            robot.gyroTurn(0.1, 315);
            robot.driveForward(0.1, 200);
            robot.gyroTurn(0.1, 180);
            //robot.dumpMarker
        }
     else if(posn == 2){
            robot.driveForward(0.1, 10);
            robot.gyroTurn(0.1, 270);
            robot.driveForward(0.1, 60);
            robot.gyroTurn(0.1, 90);
            robot.driveForward(0.1, 100);
            robot.gyroTurn(0.1, 45);
            robot.driveForward(0.1, 100);
            robot.gyroTurn(0.1, 90);

        }

    }

    /**
     * Finds and knocks gold piece.
     */
    public void findGold (){
        int posn = robot.findGoldPosition();

        if (posn == 0) {
            //fix this
        }
        else if (posn == 1) {
            //fix this
        }
        else if (posn == 2) {
            //fix this
        }
        else {
            telemetry.addLine("Can't find gold");
        }
    }


    /**
     * Drives to crater from position POSN.
     * POSN = 1 is starting spot in front of crater.
     * POSN = 2 is starting spot in front of depot.
     * POSN = 3 is from inside depot after placing marker.
     */
    public void goToCrater (int posn) {
        if (posn == 1) {
            robot.driveForward(0.1, 130);
        } else if (posn == 2) {
//            robot.driveForward(0.1,130);
            robot.gyroTurn(0.5, 135);
//            robot.driveForward(0.1,220);

        } else if (posn == 3) {
            robot.driveForward(0.1,250);

        }
    }
}

