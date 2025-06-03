package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is a Autonomous code BUT should be operate during TELEOP TIME!!!!!
 * Lever 3 auto drive
 * 遥遥领先——Richard Yu
 *
 * @author Luca Li
 * @version UNKNOWN
 *
 * To my lover -jsy
 */

@TeleOp(name = "L3-TeleOp-Code", group = "Just for show")
public class L3AutomaticTeleOp extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private AlgorithmLibrary Algorithm  = new AlgorithmLibrary(hardwareMap);

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
    private final Pose Put2 = new Pose(10,13,Math.toRadians(0));
    //Control point between GP1 and RP1
    private final Pose GP3 = new Pose(45,13,Math.toRadians(0));
    private final Pose RP3 = new Pose(40,8,Math.toRadians(0));
    private final Pose RP3Control1 = new Pose(69.8,12);
    private final Pose RP3Control2 = new Pose(61.3,7.8);
    private final Pose Push3 = new Pose(10,8,Math.toRadians(0));
    private final Pose Get1 = new Pose(27,23,Math.toRadians(0));
    //Control point between RP1 and RP2
    private final Pose PutPosition = new Pose(10,23,Math.toRadians(0));
    private final Pose FirstIntakePosition = new Pose(85,48,Math.toRadians(90));
    private final Pose FirstIntakeControlPosition = new Pose(80,34);
    private final Pose SecondIntakePosition = new Pose(70,48,Math.toRadians(90));
    private final Pose SecondIntakeControlPosition = new Pose(70,33);
    private final Pose ClimbPose = new Pose(60,96,Math.toRadians(-90));
    private final Pose ToClimbControlPose = new Pose(11,99);
    private final Pose ToClimbControlPose2 = new Pose(57,141);

    private final Pose ScorePosition = new Pose(39,70,Math.toRadians(0));
    private final Pose GetSpecPosition = new Pose(8.955,31,Math.toRadians(0));
    private Path scorePreload, park;
    private PathChain  Pickup1, Pushing3, Putting1,Putting2, Scoring, GetSpec,Intake1,Intake2,PutFrom1,PutFrom2,GetSpecFrom2;
    private int counter1 = 0,counter2 = 0,scoreCounter = 0;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(Get2)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), Get2.getHeading());

        Putting2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Get2),new Point(Put2)))
                .setLinearHeadingInterpolation(Get2.getHeading(),Put2.getHeading())
                .build();

        Pushing3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Put2),new Point(GP3)))
                .setLinearHeadingInterpolation(Get2.getHeading(),Put2.getHeading())
                .addPath(new BezierCurve(new Point(GP3),new Point(RP3Control1),new Point(RP3Control2),new Point(RP3)))
                .setLinearHeadingInterpolation(GP3.getHeading(),RP3.getHeading())
                .addPath(new BezierLine(new Point(RP3),new Point(Push3)))
                .setLinearHeadingInterpolation(RP3.getHeading(),Push3.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Push3),new Point(Get1)))
                .setLinearHeadingInterpolation(Push3.getHeading(),Get1.getHeading())
                .build();

        Putting1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Get1),new Point(PutPosition)))
                .setLinearHeadingInterpolation(Get1.getHeading(),PutPosition.getHeading())
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PutPosition),new Point(FirstIntakeControlPosition),new Point(FirstIntakePosition)))
                .setLinearHeadingInterpolation(PutPosition.getHeading(),FirstIntakePosition.getHeading())
                .build();

        PutFrom1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(FirstIntakePosition),new Point(FirstIntakeControlPosition),new Point(PutPosition)))
                .setLinearHeadingInterpolation(FirstIntakePosition.getHeading(),PutPosition.getHeading())
                .build();
        Intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PutPosition),new Point(SecondIntakeControlPosition),new Point(SecondIntakePosition)))
                .setLinearHeadingInterpolation(PutPosition.getHeading(),SecondIntakeControlPosition.getHeading())
                .build();

        PutFrom2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SecondIntakePosition),new Point(SecondIntakeControlPosition),new Point(PutPosition)))
                .setLinearHeadingInterpolation(SecondIntakePosition.getHeading(),PutPosition.getHeading())
                .build();
        GetSpecFrom2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(SecondIntakeControlPosition),new Point(SecondIntakeControlPosition),new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(SecondIntakePosition.getHeading(),GetSpecPosition.getHeading())
                .build();
        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition),new Point(ScorePosition)))
                .setLinearHeadingInterpolation(GetSpecPosition.getHeading(),ScorePosition.getHeading())
                .build();
        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ScorePosition),new Point(GetSpecPosition)))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(),GetSpecPosition.getHeading())
                .build();
        park = new Path(new BezierCurve(new Point(ScorePosition),  new Point(ToClimbControlPose), new Point(ToClimbControlPose2),new Point(ClimbPose)));
        park.setLinearHeadingInterpolation(ScorePosition.getHeading(), ClimbPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                follower.update();
                Algorithm.ArmController("Up");
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(Putting2);
                    follower.update();
                    setPathState(2);
                    break;
                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(Pushing3);
                    follower.update();
                    setPathState(3);
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Pickup1);
                    follower.update();
                    setPathState(4);
                    break;
                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Putting1);
                    follower.update();
                    setPathState(5);
                    break;
                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Intake1);
                    follower.update();
                    setPathState(6);
                    break;
                }
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(PutFrom1);
                    follower.update();
                    counter1++;
                    if(counter1<5){
                        setPathState(5);
                        break;
                    }
                    setPathState(7);
                    break;
                }
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(Intake2);
                    follower.update();
                    counter2++;
                    if((counter2)<5){
                        setPathState(8);
                        break;
                    }
                    setPathState(9);
                    break;
                }
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(PutFrom2);
                    follower.update();
                    setPathState(7);
                    break;
                }
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(GetSpecFrom2);
                    follower.update();
                    setPathState(10);
                    break;
                }
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(Scoring);
                    follower.update();
                    scoreCounter++;
                    if((scoreCounter)<11){
                        setPathState(11);
                        break;
                    }
                    setPathState(12);
                    break;
                }
            case 11:
                if(!follower.isBusy()){
                    follower.followPath(GetSpec);
                    follower.update();
                    setPathState(10);
                    break;
                }
            case 12:
                if(!follower.isBusy()){
                    follower.followPath(park);
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
        autonomousPathUpdate();

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        Algorithm.Initialize_All_For_Autonomous();
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
    }
}