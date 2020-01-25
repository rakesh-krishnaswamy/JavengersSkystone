package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;

/*
 * Title: AutoBlueCraterBase
 * Date Created: 11/23/2018
 * Date Modified: 2/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Main
 * Description: Starts on blue crater latcher
 */

@Autonomous(name = "New Blue Side", group = "Concept")
public class NewBlueSide extends LinearOpMode {
    private AutoLib autoLib;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Vuforia
        autoLib.autonArmDown();
        autoLib.calcMove(40, .3f, Constants.Direction.RIGHT);
        Constants.Coordinates coordinates = autoLib.readCoordinates();
        telemetry.addData("x", coordinates.xPosition);
        telemetry.addData("y", coordinates.yPosition);
        telemetry.update();
        if (coordinates.yPosition < 0) {
            telemetry.addData("pos", "Left");
            telemetry.update();
            autoLib.calcMove((float) (coordinates.yPosition / 10 + 17), .3f, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), .1f, Constants.Direction.RIGHT);   //when increased-moves back
//            autoLib.calcMove(30,.2f, Constants.Direction.RIGHT);
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(400);
            autoLib.autonArmUp();
            autoLib.calcTurn(3, .3f);
            autoLib.calcMove(195, .3f, Constants.Direction.FORWARD);
            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(300);
            autoLib.autonScore();
//            autoLib.calcMove(3,.2f, Constants.Direction.BACKWARD);
            autoLib.calcTurn(-3, .3f);
            autoLib.autonArmUp();
            autoLib.calcMove(250, .3f, Constants.Direction.BACKWARD);
            double distance = autoLib.getDistanceCM();
            telemetry.addData("Distance", distance);
            telemetry.update();
            if (distance > 12) {
                autoLib.calcMove((float) distance - 12, .1f, Constants.Direction.RIGHT);
            } else if (distance < 12) {
                autoLib.calcMove((float) (12 - distance), .1f, Constants.Direction.LEFT);
            }
            telemetry.addData("Distance", distance);
            telemetry.update();
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            autoLib.calcTurn(3, .3f);
            autoLib.calcMove(240, .3f, Constants.Direction.FORWARD);
            Thread.sleep(300);
            autoLib.autonArmDown();
            Thread.sleep(300);
            autoLib.autonScore();
            autoLib.autonArmUp();


        } else if (coordinates.yPosition > 0 && coordinates.yPosition < 10) {
            telemetry.addData("pos", "Center");
            telemetry.update();
//            autoLib.calcMove((float) (coordinates.yPosition / 10), .9f, Constants.Direction.FORWARD);
//            autoLib.calcMove((float) (-coordinates.xPosition / 10), .9f, Constants.Direction.RIGHT);

        } else {
            telemetry.addData("pos", "Right");
            telemetry.update();
//            autoLib.calcMove((float) (coordinates.yPosition / 10), .9f, Constants.Direction.FORWARD);
//            autoLib.calcMove((float) (-coordinates.xPosition / 10), .9f, Constants.Direction.RIGHT);
        }
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
