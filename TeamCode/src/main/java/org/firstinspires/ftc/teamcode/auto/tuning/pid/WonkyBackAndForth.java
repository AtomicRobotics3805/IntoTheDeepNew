package org.firstinspires.ftc.teamcode.auto.tuning.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.auto.FConstants;
import org.firstinspires.ftc.teamcode.auto.LConstants;
import org.firstinspires.ftc.teamcode.auto.TrajectoryFactory;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Wonky Back And Forth", group = "PIDF Tuning")
public class WonkyBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean line1 = false;
    private boolean line2 = false;
    private boolean line3 = false;
    private boolean line4 = false;
    private boolean line5 = false;
    private boolean line6 = false;
    private boolean line7 = true;


    private Follower follower;

    private Path forwards;
    private Path backwards;

    private FConstants fConstants = new FConstants();
    private LConstants lConstants = new LConstants();

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (line1) {
                line1 = false;
                line2 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine2());
            } else if (line2) {
                line2 = false;
                line3 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine3());
            } else if (line3) {
                line3 = false;
                line4 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine4());
            } else if (line4) {
                line4 = false;
                line5 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine5());
            } else if (line5) {
                line5 = false;
                line6 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine6());
            } else if (line6) {
                line6 = false;
                line7 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine7());
            } else if (line7){
                line7 = false;
                line1 = true;
                follower.followPath(TrajectoryFactory.INSTANCE.getLine1());
            }
        }
        follower.telemetryDebug(telemetryA);
    }
}
