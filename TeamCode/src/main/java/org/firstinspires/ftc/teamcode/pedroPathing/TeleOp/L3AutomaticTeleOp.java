package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.vision.GraspingCalculator;
import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;

/**
 * This is a Autonomous code BUT should be operate during TELEOP TIME!!!!!
 * Level 3 auto pilot
 *
 * 遥遥领先——Richard Yu
 *
 * @author Luca Li
 * @version 2025/6/9
 *
 * To my lover -jsy
 */

@TeleOp(name = "L3-TeleOp-Code", group = "Just for show")
public class L3AutomaticTeleOp extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static VisionGraspingAPI visionAPI;
    private static double ScorePoseNowY = ConstantMap.ScorePoseY_LeftTop;
    private static double IntakePoseNowX = ConstantMap.AutoIntakeX_LeftTop;
    private AlgorithmLibrary Algorithm;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */
    private final Pose startPose = new Pose(10, 7.7, Math.toRadians(0));
    private final Pose Get2= new Pose(27,13,Math.toRadians(0));
    private final Pose Put2 = new Pose(10,23,Math.toRadians(0));
    private final Pose Get3 = new Pose(30,7.8,Math.toRadians(0));
    private final Pose Put3 = new Pose(10,13,Math.toRadians(0));
    private final Pose Get1 = new Pose(27,23,Math.toRadians(0));
    //Control point between RP1 and RP2
    private final Pose PutPosition = new Pose(10,23,Math.toRadians(0));
    private final Pose IntakePosition = new Pose(ConstantMap.AutoIntakeX_LeftTop,ConstantMap.AutoIntakePoseY,Math.toRadians(90));
    private final Pose IntakeControlPosition = new Pose(ConstantMap.AutoIntakeX_LeftTop,ConstantMap.AutoIntakeControlPoseY);
    private final Pose ClimbPose = new Pose(60,96,Math.toRadians(-90));
    private final Pose ToClimbControlPose = new Pose(11,99);
    private final Pose ToClimbControlPose2 = new Pose(57,141);

    private final Pose ScorePose = new Pose(39,70,Math.toRadians(0));
    private final Pose GetSpecPose = new Pose(8.955,31,Math.toRadians(0));
    private Path scorePreload, park;

    private static double nextPointDistance = 0;
    private static boolean VisionIsFound = false;
    private PathChain  Pickup2,Pickup1, Putting3, Putting1,Putting2, Scoring, GetSpec,Intake1,PutFrom1,GetSpecFromIntake,Put3ToIntake;
    private int counter1 = 0,AbandonNum = 0,scoreCounter = 0;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(Get3)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Get3.getHeading());

        Putting3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Get3),new Point(Put3)))
                .setLinearHeadingInterpolation(Get3.getHeading(),Put3.getHeading())
                .build();
        Pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Put3),new Point(Get2)))
                .setLinearHeadingInterpolation(Put3.getHeading(),Get2.getHeading())
                .build();

        Putting2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Get2),new Point(Put2)))
                .setLinearHeadingInterpolation(Get2.getHeading(),Put2.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Put2),new Point(Get1)))
                .setLinearHeadingInterpolation(Put2.getHeading(),Get1.getHeading())
                .build();

        Putting1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Get1),new Point(PutPosition)))
                .setLinearHeadingInterpolation(Get1.getHeading(),PutPosition.getHeading())
                .build();
        Put3ToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(Put3),new Point(IntakeControlPosition),new Point(IntakePosition)))
                .setLinearHeadingInterpolation(Put3.getHeading(),IntakePosition.getHeading())
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PutPosition),new Point(IntakeControlPosition),new Point(IntakePosition)))
                .setLinearHeadingInterpolation(PutPosition.getHeading(),IntakePosition.getHeading())
                .build();

        PutFrom1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(IntakePosition),new Point(IntakeControlPosition),new Point(PutPosition)))
                .setLinearHeadingInterpolation(IntakePosition.getHeading(),PutPosition.getHeading())
                .build();
        GetSpecFromIntake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(IntakePosition),new Point(IntakeControlPosition),new Point(GetSpecPose)))
                .setLinearHeadingInterpolation(IntakePosition.getHeading(),GetSpecPose.getHeading())
                .build();
        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPose),new Point(ScorePose)))
                .setLinearHeadingInterpolation(GetSpecPose.getHeading(),ScorePose.getHeading())
                .build();
        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ScorePose),new Point(GetSpecPose)))
                .setLinearHeadingInterpolation(ScorePose.getHeading(),GetSpecPose.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(ScorePose),  new Point(ToClimbControlPose), new Point(ToClimbControlPose2),new Point(ClimbPose)));
        park.setLinearHeadingInterpolation(ScorePose.getHeading(), ClimbPose.getHeading());
    }
    public void buildNextScorePath(){
        if(nextPointDistance<0){
            nextPointDistance = 0;
        }
        ScorePoseNowY = ScorePoseNowY-nextPointDistance;
        //Limit the minimum position in case the robot hit he frame
        if(ScorePoseNowY<66){
            ScorePoseNowY = 66;
        }
        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(new Pose(ConstantMap.ScorePoseX,ScorePoseNowY)),new Point(GetSpecPose)))
                .setLinearHeadingInterpolation(ScorePose.getHeading(),GetSpecPose.getHeading())
                .build();

        Scoring= follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPose),new Point(new Pose(ConstantMap.ScorePoseX,ScorePoseNowY))))
                .setLinearHeadingInterpolation(GetSpecPose.getHeading(),ScorePose.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(new Pose(ConstantMap.ScorePoseX,ScorePoseNowY)),  new Point(ToClimbControlPose), new Point(ToClimbControlPose2),new Point(ClimbPose)));
        park.setLinearHeadingInterpolation(ScorePose.getHeading(), ClimbPose.getHeading());
    }
    public void buildNextIntakePath(){
        if(nextPointDistance<0){
            nextPointDistance = 0;
        }
        IntakePoseNowX = IntakePoseNowX+nextPointDistance;
        //Limit the maximum position in case the robot hit he frame
        if(IntakePoseNowX>86){
            IntakePoseNowX = 86;
        }
        Intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PutPosition),new Point(new Pose(IntakePoseNowX,ConstantMap.AutoIntakeControlPoseY)),new Point(new Pose(IntakePoseNowX,ConstantMap.AutoIntakePoseY))))
                .setLinearHeadingInterpolation(PutPosition.getHeading(),IntakePosition.getHeading())
                .build();

        PutFrom1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(new Pose(IntakePoseNowX,ConstantMap.AutoIntakePoseY)),new Point(new Pose(IntakePoseNowX,ConstantMap.AutoIntakeControlPoseY)),new Point(PutPosition)))
                .setLinearHeadingInterpolation(IntakePosition.getHeading(),PutPosition.getHeading())
                .build();
        GetSpecFromIntake = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(new Pose(IntakePoseNowX,ConstantMap.AutoIntakePoseY)),new Point(GetSpecPose)))
                .setLinearHeadingInterpolation(IntakePosition.getHeading(),GetSpecPose.getHeading())
                .build();
    }
    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                follower.update();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    VisionIntake();
                    follower.followPath(Putting3,true);
                    follower.update();
                    setPathState(2);
                    break;
                }
            case 2:
                if(!follower.isBusy()){
                    Algorithm.ForwardGrabController();
                    if(!VisionIsFound){
                        AbandonNum=2;
                        follower.followPath(Put3ToIntake,true);
                        follower.update();
                        setPathState(7);
                        break;
                    }
                    follower.followPath(Pickup2,true);
                    follower.update();
                    setPathState(3);
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    VisionIntake();
                    follower.followPath(Putting2,true);
                    follower.update();
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()){
                    Algorithm.ForwardGrabController();
                    follower.followPath(Pickup1,true);
                    follower.update();
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()){
                    VisionIntake();
                    follower.followPath(Putting1,true);
                    follower.update();
                    setPathState(6);
                    break;
                }
            case 6:
                if(!follower.isBusy()){
                    Algorithm.ForwardGrabController();
                    follower.followPath(Intake1,true);
                    follower.update();
                    setPathState(7);
                    break;
                }
            case 7:
                if(!follower.isBusy()){
                    VisionIntake();
                    follower.followPath(PutFrom1,true);
                    follower.update();
                    buildNextIntakePath();
                    counter1++;
                    if(counter1<9){
                        setPathState(6);
                        break;
                    }
                    setPathState(8);
                    break;
                }
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(Intake1,true);
                    follower.update();
                    counter1++;
                    setPathState(9);
                    break;
                }
            case 9:
                if(!follower.isBusy()){
                    VisionIntake();
                    follower.followPath(GetSpecFromIntake,true);
                    follower.update();
                    setPathState(11);
                    break;
                }/*
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(Scoring);
                    follower.update();
                    setPathState(12);
                    break;
                }*/
            case 11:
                if(!follower.isBusy()){
                    Algorithm.ForwardGrabController();
                    follower.followPath(Scoring,true);
                    follower.update();
                    scoreCounter++;
                    if((scoreCounter)<(counter1+3-AbandonNum+2/*容错*/)){
                        setPathState(12);
                        break;
                    }
                    setPathState(13);
                    break;
                }
            case 12:
                if(!follower.isBusy()){
                    VisionIntake();
                    follower.followPath(GetSpec,true);
                    follower.update();
                    buildNextScorePath();
                    setPathState(11);
                    break;
                }
            case 13:
                if(!follower.isBusy()){
                    follower.followPath(park,false);
                    follower.update();
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
    private void VisionIntake() throws InterruptedException {
        VisionGraspingAPI.VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingCalculator.GraspCalculations grasp = GraspingCalculator.calculateGrasp(result);
            if(grasp.isWithinRange){
                follower.holdPoint(follower.getPose());
                Algorithm.ClawFlag = true;
                Algorithm.BackGrabFlag = true;
                Algorithm.BackGrabAction();
                Algorithm.ForwardGrabController();
                Algorithm.performVisionGrasp(grasp.sliderServoPos, grasp.turnServoPos, grasp.rotateServoPos);
                Algorithm.IntakeSlideFlag=true;
                Algorithm.SlideController();
            }
        }
        if(!result.nextMoveDirection.equals("None")) {
            GraspingCalculator.MoveSuggestion move = GraspingCalculator.calculateMove(result);
            if(move.moveCm>0){
                nextPointDistance=0;
                return;
            }
            nextPointDistance = -move.moveCm* ConstantMap.CM_TO_INCH;
        }
    }
}