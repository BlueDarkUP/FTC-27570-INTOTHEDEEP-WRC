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
import org.firstinspires.ftc.teamcode.vision.Data.GraspingTarget;
import org.firstinspires.ftc.teamcode.vision.Interface.VisionGraspingAPI;
import org.firstinspires.ftc.teamcode.vision.CoreCalcu.VisionGraspingCalculator;
import org.firstinspires.ftc.teamcode.vision.Data.VisionTargetResult;

/**
 * This is the fucking best autonomous code in the world.
 * @author Luca Li
 * @version 2025/5
 */

@Autonomous(name = "Auto_Spec_8+Park_27570", group = "Competition")
public class SpecAutoAllVision8 extends OpMode{
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    public static VisionGraspingAPI visionAPI;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8.955, 63, Math.toRadians(0));
    private final Pose[] scorePose  = {
            new Pose(39, 76.5, Math.toRadians(0)),
            new Pose(39, 75.2, Math.toRadians(0)),
            new Pose(39, 73.8, Math.toRadians(0)),
            new Pose(39, 72.7, Math.toRadians(0)),
            new Pose(39, 71.5, Math.toRadians(0)),
            new Pose(39, 70, Math.toRadians(0)),
            new Pose(39, 68.5, Math.toRadians(0)),
            new Pose(39, 67, Math.toRadians(0))
    };


    private final Pose GetSpecPosition = new Pose(8.955,31,Math.toRadians(0));
    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));

    private int Specnum = 1;

    private Path scorePreload, park;
    private PathChain[] Scoring;
    private PathChain[] GetSpec;
    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose[0])));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose[0].getHeading());

        for(int i = 0; i < 7; i++){
            GetSpec[i] = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose[i]),new Point(GetSpecPosition)))
                    .setLinearHeadingInterpolation(scorePose[i].getHeading(),GetSpecPosition.getHeading())
                    .build();
        }
        for(int i = 0; i < 7; i++){
            Scoring[i] = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(GetSpecPosition),new Point(scorePose[i+1])))
                    .setLinearHeadingInterpolation(GetSpecPosition.getHeading(),scorePose[i+1].getHeading())
                    .build();
        }

        park = new Path(new BezierCurve(new Point(scorePose[7]),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose[7].getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate () throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
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
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    VisionIntake();
                    follower.followPath(GetSpec[Specnum-1],true);
                    follower.update();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    //Algorithm.ArmController("Down");
                    setPathState(3);
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    Algorithm.ForwardGrabController("Open");
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_TightPosition);
                    follower.followPath(Scoring[Specnum-1],true);
                    follower.update();
                    //Algorithm.ArmController("Up");
                    Specnum++;
                    if(Specnum<8) {
                        setPathState(2);
                        break;
                    }
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()) {
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    follower.followPath(park,false);
                    follower.update();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    Algorithm.ArmController("Down");
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
        visionAPI = new VisionGraspingAPI();
        visionAPI.init(hardwareMap);
        GetSpec = new  PathChain[7];
        Scoring = new PathChain[7];
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

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        if(visionAPI != null)
            visionAPI.close();
    }
    public void VisionIntake() throws InterruptedException {
        VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result,telemetry);
            if(graspTarget.isInRange){
                Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                Algorithm.ForwardGrabController("Open");
                Algorithm.performVisionGrasp(graspTarget.sliderServoPosition, graspTarget.turnServoPosition, graspTarget.rotateServoPosition);
                Algorithm.SlideController("Back");
            }
        }
    }
}

/**
 * To my lover jsy
 */