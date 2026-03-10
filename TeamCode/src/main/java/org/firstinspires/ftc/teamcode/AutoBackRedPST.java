package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechaisms.ShooterAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBackRedPST extends OpMode {
    private Limelight3A limelight3A;//limelight obj
    private Follower follower;
    private Timer pathtimer;
    private Timer opmodeTimer;
    private ShooterAuto shooter;
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
        SHOOT_BALLS_4,
        FINISH,
        STALL_A_BIT

    }
    PathState pathState;
    private final Pose startPose = new Pose(59.5,5.5,Math.toRadians(90));
    private final Pose shootPose = new Pose(59.5,15,Math.toRadians(65));
    private  final Pose endPose = new Pose(83,10.49,Math.toRadians(0));
    private  final Pose ballsPose = new Pose(97.65,10.49,Math.toRadians(0));//96
    private final Pose balls2Pose = new Pose(74.12,23.74,Math.toRadians(40));
    private final Pose picupBalls2Pose = new Pose(85.49,36.07,Math.toRadians(40));
    private  final Pose balls4Pose = new Pose(96.65,15.49,Math.toRadians(0));

    private PathChain driveFromStartToShoot, driveShootToEnd, driveEndToBalls, driveBallsToShoot, driveShootToBalls2, drivePickupBalls2, drivePickup2ToShoot, drivePickup4Balls, drive4BallsToShoot;
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
        drivePickup4Balls = follower.pathBuilder()
                .addPath(new BezierLine(endPose,balls4Pose))
                .setLinearHeadingInterpolation(endPose.getHeading(),balls4Pose.getHeading())
                .build();
        drive4BallsToShoot = follower.pathBuilder()
                .addPath(new BezierLine(balls4Pose,shootPose))
                .setLinearHeadingInterpolation(balls4Pose.getHeading(),shootPose.getHeading())
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:// first movement one
                shooter.FrontIntake.setPower(0);
                follower.followPath(driveFromStartToShoot,true);
                setPathState(PathState.STALL_A_BIT);
                break;
            case STALL_A_BIT:
                if(!follower.isBusy() && pathtimer.getElapsedTimeSeconds() > 3.5) {
                    shooter.setShooterState(ShooterAuto.ShooterState.SHOOT);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                //is folower don with path

                if(!follower.isBusy() && pathtimer.getElapsedTimeSeconds()  > 2.75){
                    telemetry.addLine("done path 1");
                    follower.followPath(driveShootToEnd);
                    setPathState(PathState.DRIVE_SHOOT_END);
                }
                break;
            case DRIVE_SHOOT_END:
                //all done
                shooter.setShooterState(ShooterAuto.ShooterState.IDLE);
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveEndToBalls);
                    setPathState(PathState.DRIVE_END_BALLS);
                    shooter.FrontIntake.setPower(1);
                }
                break;
            case DRIVE_END_BALLS:
                //all done
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveBallsToShoot);
                    setPathState(PathState.DRIVE_BALLS_SHOOT);
                    shooter.FrontIntake.setPower(0);
                }
                break;
            case DRIVE_BALLS_SHOOT:
                //all done
                if(!follower.isBusy() && pathtimer.getElapsedTimeSeconds()  > 2){
                    telemetry.addLine ("done all paths");
                    shooter.setShooterState(ShooterAuto.ShooterState.SHOOT);
                    setPathState(PathState.SHOOT_BALLS2);
                }
                break;
            case SHOOT_BALLS2:
                if(pathtimer.getElapsedTimeSeconds()  > 3.25){
                    telemetry.addLine ("done all paths");
                    follower.followPath(driveShootToBalls2);
                    shooter.setShooterState(ShooterAuto.ShooterState.IDLE);
                    setPathState(PathState.DRIVE_SHOOT_BALLS2);
                }

                break;

            case DRIVE_SHOOT_BALLS2:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    setPathState(PathState.PICKUP_BALLS2);
                    follower.followPath(drivePickupBalls2);
                    shooter.FrontIntake.setPower(1);
                }
                break;
            case PICKUP_BALLS2:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    follower.followPath(drivePickup2ToShoot);
                    setPathState(PathState.DRIVE_SHOOT_BALLS_3);
                    shooter.FrontIntake.setPower(0);
                }
                break;
            case DRIVE_SHOOT_BALLS_3:
                if(!follower.isBusy() && pathtimer.getElapsedTimeSeconds()  > 2){
                    telemetry.addLine ("done all paths");

                    shooter.setShooterState(ShooterAuto.ShooterState.SHOOT);


                    setPathState(PathState.SHOOT_BALLS_3);
                }
                break;
            case SHOOT_BALLS_3:
                if(pathtimer.getElapsedTimeSeconds()  > 3){
                    telemetry.addLine ("done all paths");
                    setPathState(PathState.DRIVE_GRAB_BALLS_3);
                    shooter.setShooterState(ShooterAuto.ShooterState.IDLE);
                    follower.followPath(driveShootToEnd);
                    shooter.FrontIntake.setPower(1);

                }

                break;
            case DRIVE_GRAB_BALLS_3:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    shooter.FrontIntake.setPower(1);
                    setPathState(PathState.GRAB_BALLS_3);
                    follower.followPath(drivePickup4Balls);
                }
                break;
            case GRAB_BALLS_3:
                if(!follower.isBusy() || pathtimer.getElapsedTimeSeconds()  > 1){
                    telemetry.addLine ("done all paths");
                    shooter.FrontIntake.setPower(0);
                    setPathState(PathState.SHOOT_BALLS_4);
                    follower.followPath(drive4BallsToShoot);
                }
                break;
            case SHOOT_BALLS_4:
                if(!follower.isBusy()){
                    telemetry.addLine ("done all paths");
                    shooter.setShooterState(ShooterAuto.ShooterState.SHOOT);
                    setPathState(PathState.FINISH);
                }
                break;
            case FINISH:
                if(pathtimer.getElapsedTimeSeconds()  > 3){
                    telemetry.addLine ("done all paths");
                    shooter.setShooterState(ShooterAuto.ShooterState.IDLE);
                    follower.followPath(driveShootToBalls2);

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
        shooter = new ShooterAuto(hardwareMap);
        //opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        shooter.FrontIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        shooter.FrontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.FrontIntake.setPower(0);
        shooter.setShooterState(ShooterAuto.ShooterState.IDLE);
        shooter.targetSpeed = 840;
        telemetry.addData("shooterSpeedLeft",shooter.leftShooter.getVelocity());
        telemetry.addData("shooterSpeedRight",shooter.rightShooter.getVelocity());
    }

    @Override
    public void start() {
        limelight3A.start();
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        double rot = 0;
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null & llResult.isValid()){
            rot = llResult.getTx();
        }else{
            rot = -1;
        }
        if(isPathingOn) {
            follower.update();
        }
        shooter.update();
        statePathUpdate();
        telemetry.addData("lltx",rot);
        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathtimer.getElapsedTimeSeconds());
    }
    @Override
    public void stop(){
        isPathingOn = false;
    }

}
