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

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.vision.GraspingCalculator;
import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;

/**
 * This is the fucking best autonomous code in the world.
 * @author Luca Li
 * @version 2025/5
 */

@Autonomous(name = "Auto_Spec_8+Park_27570", group = "Competition")
public class SpecAutoAllVision8 extends OpMode {
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    public static VisionGraspingAPI visionAPI;
    private static double ScorePoseNowY = ConstantMap.ScorePoseY_LeftTop;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;

    private final Pose startPose = new Pose(8.955, 63, Math.toRadians(0));
    private final Pose scorePose = new Pose(ConstantMap.ScorePoseX, ConstantMap.ScorePoseY_LeftTop, Math.toRadians(0));


    private final Pose GetSpecPosition = new Pose(8.955, 31, Math.toRadians(0));
    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));

    private int Specnum = 1;
    private static double nextPointDistance = 0;

    private Path scorePreload, park;
    private PathChain Scoring;
    private PathChain GetSpec;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GetSpecPosition.getHeading())
                .build();

        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition), new Point(scorePose)))
                .setLinearHeadingInterpolation(GetSpecPosition.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void buildNextPath() {
        ScorePoseNowY = ScorePoseNowY - nextPointDistance;
        //Limit the minimum position in case the robot hit he frame
        if (ScorePoseNowY < 66) {
            ScorePoseNowY = 66;
        }
        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(new Pose(ConstantMap.ScorePoseX, ScorePoseNowY)), new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GetSpecPosition.getHeading())
                .build();

        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition), new Point(new Pose(ConstantMap.ScorePoseX, ScorePoseNowY))))
                .setLinearHeadingInterpolation(GetSpecPosition.getHeading(), scorePose.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(new Pose(ConstantMap.ScorePoseX, ScorePoseNowY)), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                follower.update();
                //Algorithm.ArmController("Up");

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
                if (!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    VisionIntake();
                    follower.followPath(GetSpec, true);
                    follower.update();
                    buildNextPath();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    //Algorithm.ArmController("Down");
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()) {
                    Algorithm.ForwardGrabController();
                    Algorithm.BackGrabAction();
                    follower.followPath(Scoring, true);
                    follower.update();
                    //Algorithm.ArmController("Up");
                    Specnum++;
                    if (Specnum < 8) {
                        setPathState(2);
                        break;
                    }
                    setPathState(4);
                    break;
                }
            case 4:
                if (!follower.isBusy()) {
                    Algorithm.BackGrabAction();
                    follower.followPath(park, false);
                    follower.update();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    Algorithm.ArmController();
                    setPathState(-1);
                    break;
                }
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Algorithm = new AlgorithmLibrary(hardwareMap);
        Algorithm.Initialize_All_For_Autonomous();
        visionAPI = new VisionGraspingAPI();
        visionAPI.init(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        if (visionAPI != null)
            visionAPI.close();
    }

    private void VisionIntake() throws InterruptedException {
        VisionGraspingAPI.VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingCalculator.GraspCalculations grasp = GraspingCalculator.calculateGrasp(result);
            if (grasp.isWithinRange) {
                Algorithm.ClawFlag = true;
                Algorithm.BackGrabFlag = true;
                Algorithm.BackGrabAction();
                Algorithm.ForwardGrabController();
                Algorithm.performVisionGrasp(grasp.sliderServoPos, grasp.turnServoPos, grasp.rotateServoPos);
                Algorithm.IntakeSlideFlag = true;
                Algorithm.SlideController();
            }
            if(!result.nextMoveDirection.equals("None")) {
                GraspingCalculator.MoveSuggestion move = GraspingCalculator.calculateMove(result);
                if(move.moveCm>0){
                    nextPointDistance=0;
                    return;
                }
                nextPointDistance = -move.moveCm * ConstantMap.CM_TO_INCH;
            }
        }
    }
}
/**
 * To my lover jsy
 */