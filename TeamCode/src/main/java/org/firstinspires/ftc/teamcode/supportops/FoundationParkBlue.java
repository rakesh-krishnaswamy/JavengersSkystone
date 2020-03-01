package org.firstinspires.ftc.teamcode.supportops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;

/*
 * Title: CalcTurn Test
 * Date Created: 2/13/2019
 * Date Modified: 2/22/2019
 * Author: Poorvi
 * Type: Support
 * Description: This will test if the robot can actually turn
 */

@Autonomous(group = "Foundation Park Blue Side")
public class FoundationParkBlue extends LinearOpMode {
    private AutoLib autoLib;


    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("about to move", "initialized");
        telemetry.update();
//        Thread.sleep(20000);
        autoLib.calcMove(20, .4f, Constants.Direction.LEFT);
        autoLib.calcMove(60, .3f, Constants.Direction.BACKWARD);
        autoLib.calcMove(17, .05f, Constants.Direction.BACKWARD);
        autoLib.calcMove(3, .05f, Constants.Direction.BACKWARD);
        autoLib.latchServoFoundation();
        Thread.sleep(1000);
        autoLib.calcMove(63, .4f, Constants.Direction.FORWARD);
        autoLib.calcTurn(100, 1f);
        Thread.sleep(400);
        autoLib.restServoFoundation();
        Thread.sleep(400);
        autoLib.calcMove(30, .2f, Constants.Direction.LEFT);
        autoLib.calcMove(100, .2f, Constants.Direction.FORWARD);
        telemetry.addData("Just moved", "finished moving");
        telemetry.update();
    }


    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib(this);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
