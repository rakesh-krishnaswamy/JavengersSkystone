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

@Autonomous(group = "Foundation Park Wall Red")
public class FoundationParkWallRed extends LinearOpMode {
    private AutoLib autoLib;


    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("about to move","initialized");
        telemetry.update();
        autoLib.calcMove(5,.5f, Constants.Direction.BACKWARD);
        autoLib.calcMove(200,.5f, Constants.Direction.LEFT);
        autoLib.calcMove(50,.5f, Constants.Direction.BACKWARD);
        autoLib.calcMove(10,.2f, Constants.Direction.BACKWARD);
        Thread.sleep(300);
        autoLib.latchServoFoundation();
        Thread.sleep(1000);
        autoLib.calcMove(90, 1f, Constants.Direction.FORWARD);
        autoLib.recieveServoArm();
        autoLib.calcTurn(170,1f);
        autoLib.restServoFoundation();
        autoLib.calcMove(100, 1f, Constants.Direction.FORWARD);
        telemetry.addData("Just moved","finished moving");
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
