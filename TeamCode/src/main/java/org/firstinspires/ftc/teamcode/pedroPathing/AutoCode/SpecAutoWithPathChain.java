package org.firstinspires.ftc.teamcode.pedroPathing.AutoCode;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * Hope it could run smoothly……
 * This is the fucking worst autonomous code in the world [>_<]
 * @author Luca Li
 * @version 2025/5
 */

@Autonomous(name = "Auto_Spec_5+PARK_27570_WithPathChain", group = "Competition")
public class SpecAutoWithPathChain extends OpMode{
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8.955, 63, Math.toRadians(0));

    private final Pose FscorePose = new Pose(39, 70, Math.toRadians(0));

    private final Pose Push1Pose = new Pose(20, 23, Math.toRadians(0));
    private final Pose Push1Control1Pose = new Pose(1.6,22.6);
    private final Pose Push1Control2Pose = new Pose(43.4,32.2);
    private final Pose Push1Control3Pose = new Pose(78.6,53.3);
    private final Pose Push1Control4Pose = new Pose(66.5,13.7);
    private final Pose Push1Control5Pose = new Pose(55.5,24.7);
    private final Pose Push2Pose = new Pose(20,14,Math.toRadians(0));

    private final Pose Push2Control1Pose = new Pose(97.2,24.2);
    private final Pose Push2Control2Pose = new Pose(24.7,17.2);
    private final Pose Push2Control3Pose = new Pose(94.9,13);
    private final Pose Push3Pose = new Pose(20,8.5,Math.toRadians(0));

    private final Pose Push3Control1Pose = new Pose(98.1,13.7);
    private final Pose Push3Control2Pose = new Pose(21.7,10.7);
    private final Pose Push3Control3Pose = new Pose(97.4,7.6);
    private final Pose GetspecControlPosition = new Pose(20,31);
    private final Pose GetSpecPosition = new Pose(8.955,31,Math.toRadians(0));
    private final Pose scorePose = new Pose(39,70,Math.toRadians(0));
    private final Pose scorePoseControlPose = new Pose(25,31,Math.toRadians(0));
    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));

    private int Specnum = 1;

    private Path scorePreload, park;
    private PathChain Push1,Push2,Push3,PushToGet,Scoring,Getspec,Push3Together;


    public void buildPaths() {



        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(FscorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), FscorePose.getHeading());
        Push3Together = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(FscorePose),new Point(Push1Control1Pose),new Point(Push1Control2Pose),new Point(Push1Control3Pose),new Point(Push1Control4Pose),new Point(Push1Control5Pose),new Point(Push1Pose)))
                .setLinearHeadingInterpolation(FscorePose.getHeading(),Push1Pose.getHeading())
                .addParametricCallback(0.5, () -> {
                    try {
                        Algorithm.ArmController();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })

                .addPath(new BezierCurve(new Point(Push1Pose),new Point(Push2Control1Pose),new Point(Push2Control2Pose),new Point(Push2Control3Pose),new Point(Push2Pose)))
                .setLinearHeadingInterpolation(Push1Pose.getHeading(),Push2Pose.getHeading())

                .addPath(new BezierCurve(new Point(Push2Pose),new Point(Push3Control1Pose),new Point(Push3Control2Pose),new Point(Push3Control3Pose),new Point(Push3Pose)))
                .setLinearHeadingInterpolation(Push2Pose.getHeading(),Push3Pose.getHeading())

                .addPath(new BezierCurve(new Point(Push3Pose),new Point(GetspecControlPosition),new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(Push3Pose.getHeading(),GetSpecPosition.getHeading())

                .build();

        Push1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(FscorePose),new Point(Push1Control1Pose),new Point(Push1Control2Pose),new Point(Push1Control3Pose),new Point(Push1Control4Pose),new Point(Push1Control5Pose),new Point(Push1Pose)))
                .setLinearHeadingInterpolation(FscorePose.getHeading(),Push1Pose.getHeading())
                .build();



        Push2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(Push1Pose),new Point(Push2Control1Pose),new Point(Push2Control2Pose),new Point(Push2Control3Pose),new Point(Push2Pose)))
                .setLinearHeadingInterpolation(Push1Pose.getHeading(),Push2Pose.getHeading())
                .build();


        Push3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(Push2Pose),new Point(Push3Control1Pose),new Point(Push3Control2Pose),new Point(Push3Control3Pose),new Point(Push3Pose)))
                .setLinearHeadingInterpolation(Push2Pose.getHeading(),Push3Pose.getHeading())
                .build();

        PushToGet = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(Push3Pose),new Point(GetspecControlPosition),new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(Push3Pose.getHeading(),GetSpecPosition.getHeading())
                .build();

        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition),new Point(scorePose)))
                .setLinearHeadingInterpolation(GetSpecPosition.getHeading(),scorePose.getHeading())
                .build();
        Getspec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),new Point(GetSpecPosition)))
                .addParametricCallback(0.4, () -> {
                    try {
                        Algorithm.ArmController();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .setLinearHeadingInterpolation(scorePose.getHeading(),GetSpecPosition.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(scorePose),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate () throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                follower.update();
                Algorithm.ArmController();
                // Thread.sleep(200);

                //Algorithm.BackGrabAction(ConstantMap.BackGrab_LaxPosition);
                setPathState(2);
                break;
            /*case 1:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload,false);
                    follower.update();
                    setPathState(2);
                    break;
                }*/
            case 2:
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    follower.followPath(Push3Together, true);
                    follower.update();
                    setPathState(6);
                    break;
                }/*
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Push2,false);
                    follower.update();
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Push3,false);
                    follower.update();
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(PushToGet,true);
                    follower.update();
                    setPathState(6);
                    break;
                }*/
                //Scoring loop
            case 6:
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    Thread.sleep(170);
                    follower.followPath(Scoring,true);
                    follower.update();
                    Algorithm.ArmController();
                    Specnum += 1;
                    if (Specnum < 5) {
                        setPathState(7);
                        break;
                    }
                    setPathState(8);
                    break;
                }
            case 7:
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    follower.followPath(Getspec, true);
                    follower.update();
                    setPathState(6);
                    break;
                }
            case 8:
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    follower.followPath(park,false);
                    follower.update();
                    Thread.sleep(700);
                    Algorithm.ArmController();
                    setPathState(-1);
                    break;
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Algorithm = new AlgorithmLibrary(hardwareMap);
        Algorithm.Initialize_All_For_Autonomous();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
/**
 * To my lover jsy
 */