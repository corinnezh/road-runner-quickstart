package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

public class Robot {
    public Intake intake;
    public SampleMecanumDrive drive;
    public MultipleTelemetry telemetry;
    NanoClock clock = NanoClock.system();
    private double lastLoop = 0;
    private double thisLoop = 0;

    HardwareMap hardwareMap;

    private List<LynxModule> connectedHubs;

    private SubsystemScheduler driveScheduler = new SubsystemScheduler(1);


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        this.connectedHubs = hardwareMap.getAll(LynxModule.class);


        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update(){
        thisLoop = clock.seconds();                   // get timestamp for this loop
        refreshBulkCache(0);             //2ms to execute bulk read on control hub
        drive.updatePoseEstimate();                   // this is cheap
        if (driveScheduler.updateDesired(thisLoop)) drive.updateDriveCommand(); // 4*2 = 8ms to update

        updateTelemetry();
    }

    public void updateRobotWhileDriveBusy() {
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            update();
        }
    }

    public void updateRobotUntilTimer(double timerReference) {
        while(!Thread.currentThread().isInterrupted() && isAsyncTimerFinished(timerReference)) {
            update();
        }
    }

    public void executeTrajectory(Trajectory trajectory){
        drive.followTrajectoryAsync(trajectory);
        updateRobotWhileDriveBusy();
    }

    public void updateTelemetry() {
        telemetry.addData("Robot Position", drive.getPoseEstimate());
        telemetry.addData("Traj Status", drive.isBusy());

        telemetry.addData("Control Loop Time", "%.1f ms", queryLoopTime());

        telemetry.update();
    }

    public double queryModuleCurrent(int moduleIndex) {
        return hardwareMap.getAll(LynxModule.class).get(moduleIndex).getCurrent(CurrentUnit.AMPS);
    }

    public double createTimerReference(double timerLength){
        return timerLength + clock.seconds();
    }

    public boolean isAsyncTimerFinished(double timerReference){
        return timerReference > clock.seconds();
    }

    public void blockingTimer(double timerReference){
        updateRobotUntilTimer(timerReference);
    }

    private double queryLoopTime() {
        if(lastLoop!=0){
            double loopTime = (thisLoop-lastLoop)*1000; //return in milliseconds
            lastLoop = thisLoop;
            return loopTime;
        } else {
            lastLoop = thisLoop;
            return 1;
        }
    }

    private void refreshBulkCache() {
        for (LynxModule hub : connectedHubs) {
            hub.clearBulkCache();
        }
    }

    private void refreshBulkCache(int moduleNumber){
        if(connectedHubs.size()>=moduleNumber) connectedHubs.get(moduleNumber).clearBulkCache();
    }
}
