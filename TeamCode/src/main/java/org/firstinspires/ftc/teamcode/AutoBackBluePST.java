package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class AutoBackBluePST extends OpMode {
    private Limelight3A limelight3A;//limelight obj
    private Follower follower;
    private Timer pathtimer;
    private Timer opmodeTimer;
    private DcMotor FrontIntake;
    boolean isPathingOn = true;

    public enum  PathState{
        //startpos-endpos
        //drive>movement state
        //shoot atempt to score
         DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_END,
        DRIVE_END_BALLS,
        DRIVE_BALLS_SHOOT,
        SHOOT_BALLS2,
        DRIVE_SHOOT_BALLS2,
        PICKUP_BALLS2,
        DRIVE_SHOOT_BALLS_3,
        SHOOT_BALLS_3,
        DRIVE_GRAB_BALLS_3,
        GRAB_BALLS_3,

    }
    PathState pathState;
    private final Pose startPose = new Pose(59.5,5.5,Math.toRadians(90));
    private final Pose shootPose = new Pose(59.5,16,Math.toRadians(113));
    private  final Pose endPose = new Pose(29,11.49,Math.toRadians(180));
    private  final Pose ballsPose = new Pose(20.35,11.49,Math.toRadians(180));
    private final Pose balls2Pose = new Pose(48.88,23.74,Math.toRadians(140));
    private final Pose picupBalls2Pose = new Pose(32.51,36.07,Math.toRadians(140));

    private PathChain driveFromStartToShoot, driveShootToEnd, driveEndToBalls, driveBallsToShoot, driveShootToBalls2, drivePickupBalls2, drivePickup2ToShoot;
    public void buildPaths(){
        //put in cords for star and end  pose
        driveFromStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
                .build();
        driveEndToBalls = follower.pathBuilder()
                .addPath(new BezierLine(endPose,ballsPose))
                .setLinearHeadingInterpolation(endPose.getHeading(),ballsPose.getHeading())
                .build();
        driveBallsToShoot = follower.pathBuilder()
                .addPath(new BezierLine(ballsPose,shootPose))
                .setLinearHeadingInterpolation(ballsPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootToBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,balls2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), balls2Pose.getHeading())
                .build();
        drivePickupBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(balls2Pose,picupBalls2Pose))
                .setLinearHeadingInterpolation(balls2Pose.getHeading(), picupBalls2Pose.getHeading())
                .build();
        drivePickup2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(picupBalls2Pose,shootPose))
                .setLinearHeadingInterpolation(picupBalls2Pose.getHeading(), shootPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:// first movement one
                follower.followPath(driveFromStartToShoot,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //is folower don with path
                if(!follower.isBusy() && pathtimer.getElapsedTimeSeconds()  > 5){
                    telemetry.addLine("done path 1");
                    follower.followPath(driveShootToEnd);
                    setPathState(PathState.DRIVE_SHOOT_END);
                }
                break;
            case DRIVE_SHOOT_END:
                //all done
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveEndToBalls);
                    setPathState(PathState.DRIVE_END_BALLS);
                    FrontIntake.setPower(1);
                }
                break;
            case DRIVE_END_BALLS:
                //all done
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveBallsToShoot);
                    setPathState(PathState.DRIVE_BALLS_SHOOT);
                    FrontIntake.setPower(0);
                }
                break;
            case DRIVE_BALLS_SHOOT:
                //all done
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    setPathState(PathState.SHOOT_BALLS2);
                }
                break;
            case SHOOT_BALLS2:
                if(pathtimer.getElapsedTimeSeconds()  > 2){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveShootToBalls2);
                    setPathState(PathState.DRIVE_SHOOT_BALLS2);
                }

                break;

            case DRIVE_SHOOT_BALLS2:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    setPathState(PathState.PICKUP_BALLS2);
                    follower.followPath(drivePickupBalls2);
                    FrontIntake.setPower(1);
                }
                break;
            case PICKUP_BALLS2:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(drivePickup2ToShoot);
                    setPathState(PathState.DRIVE_SHOOT_BALLS_3);
                    FrontIntake.setPower(0);
                }
                break;
            case DRIVE_SHOOT_BALLS_3:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");

                    setPathState(PathState.SHOOT_BALLS_3);
                }
                break;
            case SHOOT_BALLS_3:
                if(pathtimer.getElapsedTimeSeconds()  > 2){
                    telemetry.addLine ("done all paths");
                    setPathState(PathState.DRIVE_GRAB_BALLS_3);
                    follower.followPath(driveShootToEnd);

                }

                break;
            case DRIVE_GRAB_BALLS_3:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    FrontIntake.setPower(1);
                    setPathState(PathState.GRAB_BALLS_3);
                    follower.followPath(driveEndToBalls);
                }
                break;
            case GRAB_BALLS_3:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    FrontIntake.setPower(0);

                    follower.followPath(driveBallsToShoot);
                }
                break;
            default:
                telemetry.addLine("no state");
                break;
        }
    }

    public void setPathState( PathState newState){
        pathState = newState;
        pathtimer.resetTimer();
    }
    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(4);//1 is green
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        opmodeTimer = new Timer();
        //opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        FrontIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontIntake.setPower(0);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        if(isPathingOn) {
            follower.update();
        }
        statePathUpdate();
        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathtimer.getElapsedTimeSeconds());
    }

}
