package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;
import org.opencv.core.Point;


/*
 * Title: AutoBlueCraterBase
 * Date Created: 11/23/2018
 * Date Modified: 2/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Main
 */

@Autonomous(name = "BlueSideOpenCv", group = "Concept")
public class BlueSideOpenCv extends LinearOpMode {
    private AutoLib autoLib;
    // Description: Starts on blue crater latcher

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        float fullPower = 1f;
        float fastPower = 0.8f;
        float mediumPower = .7f; //.6f
        float slowPower = 0.3f;
        float verySlowPower = 0.1f;
        double distance = 0;
        float foundationDistance = 5f;  //2
        float defaultMaxDistance = 15f;
        int moveToStoneDistance = 60;
        int backAndForthStoneDistance = 7; //4
        int stoneLength = 20;
        int stoneToFoundationDistance = 170;    //230

        // Vuforia
        telemetry.addData("pos", autoLib.getPipeline().getDetectedPosition());
        telemetry.update();
        autoLib.autonArmDown();
//        autoLib.calcMove(45, mediumPower, Constants.Direction.RIGHT);
//        Constants.Coordinates coordinates = autoLib.readCoordinates();
//        autoLib.getPipeline().getDetectedPosition();
//        distance = autoLib.getDistanceCM();


        if (autoLib.getPipeline().getDetectedPosition() == 0) {
            telemetry.addData("pos", "Left");
            telemetry.update();
            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
            autoLib.calcMove(stoneLength, slowPower, Constants.Direction.FORWARD);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
//            autoLib.rampMove(stoneToFoundationDistance, slowPower, Constants.Direction.FORWARD, true);  // move forward towards foundation
            autoLib.calcMove(stoneToFoundationDistance, mediumPower, Constants.Direction.FORWARD);  // move forward towards foundation
            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - (2 * foundationDistance)), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(85, fullPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(110, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 1) {
            telemetry.addData("pos", "Center");
            telemetry.update();

            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(stoneToFoundationDistance + stoneLength, fullPower, Constants.Direction.FORWARD);
            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - (2 * foundationDistance)), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(85, fullPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(110, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 2) {
            telemetry.addData("pos", "Right");
            telemetry.update();
            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
            autoLib.calcMove(stoneLength, slowPower, Constants.Direction.BACKWARD);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(stoneToFoundationDistance + (stoneLength * 2), fullPower, Constants.Direction.FORWARD);
//            autoLib.calcMove(backAndForthStoneDistance + foundationAdjustedDistance, slowPower, Constants.Direction.RIGHT);
            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - (2 * foundationDistance)), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(85, fullPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(110, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);
        }
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        //        autoLib = new AutoLib(this, new Point[]{new Point(194, 184), new Point(275, 160), new Point(194, 353), new Point(275, 331)});
        autoLib = new AutoLib(this, new Point[]{new Point(181, 211), new Point(264, 242), new Point(181, 376), new Point(264, 408)});
        autoLib.autonScore();
        autoLib.restServoFoundation();
        autoLib.grabCapstone();
        autoLib.scoringSlideRest();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
