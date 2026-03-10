package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Disabled
public class testPathingAuto extends OpMode {
    private Limelight3A limelight3A;//limelight obj
    private Follower follower;
    private Timer pathtimer;
    private Timer opmodeTimer;

    public enum  PathState{
        //startpos-endpos
        //drive>movement state
        //shoot atempt to score
         DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_END
    }
    PathState pathState;
    private final Pose startPose = new Pose(21.94,122.795,Math.toRadians(137));
    private final Pose shootPose = new Pose(52.3321,92.5342,Math.toRadians(137));
    private  final Pose endPose = new Pose(59.2,108.09,Math.toRadians(90));
    private PathChain driveFromStartToShoot, driveShootToEnd;
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
    }
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
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
                }
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
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathtimer.getElapsedTimeSeconds());
    }

}
