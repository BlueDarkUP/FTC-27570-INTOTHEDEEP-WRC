package org.firstinspires.ftc.teamcode.pedroPathing.AutoCode;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;

/**
 * This is the fucking best autonomous code in the world.
 * @author Luca Li
 * @version 2025/5
 */

@Autonomous(name = "Auto_Spec_7+Park_27570", group = "Competition")
public class SpecAutoBY27570 extends OpMode{
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private final Pose startPose = new Pose(8.955, 63, Math.toRadians(0));
    private final Pose GP1= new Pose(50,35,Math.toRadians(0));
    private final Pose RP1 = new Pose(50,23,Math.toRadians(0));
    //Control point between GP1 and RP1
    private final Pose RP1C1 = new Pose(64.7,37,Math.toRadians(9));
    private final Pose RP1C2 = new Pose(61.8,22.5,Math.toRadians(0));
    private final Pose Push1 = new Pose(20,23,Math.toRadians(0));
    private final Pose RP2 = new Pose(50,14,Math.toRadians(0));
    //Control point between RP1 and RP2
    private final Pose RP2C1 = new Pose(64.7,22.9,Math.toRadians(0));
    private final Pose RP2C2 = new Pose(60.4,13.1,Math.toRadians(0));
    private final Pose Push2 = new Pose(20,14,Math.toRadians(0));


    private final Pose[] scorePose  = {
            new Pose(39, 76, Math.toRadians(0)),
            new Pose(39, 73, Math.toRadians(0)),
            new Pose(39, 71, Math.toRadians(0)),
            new Pose(39, 69, Math.toRadians(0)),
            new Pose(39, 69, Math.toRadians(0)),
            new Pose(39, 69, Math.toRadians(0)),
            new Pose(39, 69, Math.toRadians(0))
    };


    private final Pose GetSpecPosition = new Pose(8.955,31,Math.toRadians(0));
    private final Pose GSControlP = new Pose(20.8,32.7);
    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));

    private int Specnum = 0;

    private Path scorePreload, park;
    private PathChain Push;
    private PathChain[] Scoring; // 声明了数组，但尚未初始化（为null）
    private PathChain[] GetSpec; // 声明了数组，但尚未初始化（为null）

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose[0])));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose[0].getHeading());
        Push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition),new Point(GP1)))
                .setLinearHeadingInterpolation(GetSpecPosition.getHeading(),GP1.getHeading())

                .addPath(new BezierCurve(new Point(GP1),new Point(RP1C1),new Point(RP1C2),new Point(RP1)))
                .setLinearHeadingInterpolation(GP1.getHeading(),RP1.getHeading())

                .addPath(new BezierLine(new Point(RP1),new Point(Push1)))
                .setLinearHeadingInterpolation(RP1.getHeading(),Push1.getHeading())

                .addPath(new BezierLine(new Point(Push1),new Point(RP1)))
                .setLinearHeadingInterpolation(Push1.getHeading(),RP1.getHeading())

                .addPath(new BezierCurve(new Point(RP1),new Point(RP2C1),new Point(RP2C2),new Point(RP2)))
                .setLinearHeadingInterpolation(RP1.getHeading(),RP2.getHeading())

                .addPath(new BezierLine(new Point(RP2),new Point(Push2)))
                .setLinearHeadingInterpolation(RP2.getHeading(),Push2.getHeading())

                .addPath(new BezierCurve(new Point(Push2),new Point(GSControlP),new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(Push2.getHeading(),GetSpecPosition.getHeading())
                .build();
        for(int i = 0; i < 5; i++){
            // 之前报错的第101行：当 GetSpec 为 null 时，尝试访问 GetSpec[i] 会导致 NullPointerException
            GetSpec[i] = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(scorePose[i]),new Point(GetSpecPosition)))
                    .setLinearHeadingInterpolation(scorePose[i].getHeading(),GetSpecPosition.getHeading())
                    .build();
        }
        for(int i = 0; i < 4; i++){
            // 同样，如果 Scoring 为 null，这里也会报错
            Scoring[i] = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(GetSpecPosition),new Point(scorePose[i+1])))
                    .setLinearHeadingInterpolation(GetSpecPosition.getHeading(),scorePose[i+1].getHeading())
                    .build();
        }

        park = new Path(new BezierCurve(new Point(scorePose[3]),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose[3].getHeading(), parkPose.getHeading());
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
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    Specnum++;
                    follower.followPath(GetSpec[Specnum-1],true);
                    follower.update();
                    //Algorithm.SlideController("Back");
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    //Algorithm.ArmController("Down");
                    if(Specnum<3) {
                        setPathState(2);
                        break;
                    }
                    setPathState(3);
                    break;
                }
            case 2:
                if(!follower.isBusy()){
                    //Algorithm.ForwardGrabController("Open");
                    //Algorithm.BackGrabAction(ConstantMap.BackGrab_TightPosition);
                    follower.followPath(Scoring[Specnum-1],true);
                    follower.update();
                    //Algorithm.ArmController("Up");
                    setPathState(1);
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Push);
                    follower.update();
                    setPathState(5);
                }
            case 4:
                if(!follower.isBusy()) {
                    //Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    follower.followPath(GetSpec[3],true);
                    follower.update();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    //Algorithm.ArmController("Down");
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()){
                    //Algorithm.BackGrabAction(ConstantMap.BackGrab_TightPosition);
                    follower.followPath(Scoring[2],true);
                    follower.update();
                    //Algorithm.ArmController("Up");
                    Specnum++;
                    if(Specnum<7) {
                        setPathState(4);
                        break;
                    }
                    setPathState(6);
                    break;
                }
            case 6:
                if(!follower.isBusy()) {
                    //Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    follower.followPath(park,false);
                    follower.update();
                    Thread.sleep(ConstantMap.SleepMSAfterScoring);
                    //Algorithm.ArmController("Down");
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // --- 核心修复：在这里初始化数组 ---
        // GetSpec 数组在 buildPaths() 中被循环 i < 5，所以需要大小为 5
        GetSpec = new PathChain[5];
        // Scoring 数组在 buildPaths() 中被循环 i < 4，所以需要大小为 4
        Scoring = new PathChain[4];
        // ---------------------------------

        buildPaths(); // 现在 buildPaths() 可以安全地为数组元素赋值了
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
    }
}

/**
 * To my lover jsy [^_^]
 */