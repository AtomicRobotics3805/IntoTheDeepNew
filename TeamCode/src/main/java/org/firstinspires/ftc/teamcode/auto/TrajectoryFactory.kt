package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathBuilder
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point

@Config
object TrajectoryFactory {

    // region Poses


    // region Sample
    @JvmField
    var sampleStartPos = Point(9.0, 105.0, Point.CARTESIAN)

    @JvmField
    var sampleScorePose = Point(18.0, 126.0)
    @JvmField
    var firstSamplePos = Point(30.0, 121.0)
    @JvmField
    var secondSamplePos = Point(30.0, 131.5)
    @JvmField
    var thirdSamplePos = Point(32.348, 130.435)
    @JvmField
    var sampleParkPos = Point(61.148, 97.252)

    // endregion
    
    // endregion

    // region Paths

    var builder: PathBuilder = PathBuilder()

    // region Sample


    var testPath: PathChain = builder
        .addPath(
            BezierCurve(
                Point(0.0, 0.0, Point.CARTESIAN),
                Point(4.0, 4.0, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0.0))
        .build()

    var scorePreloadSample: PathChain = builder
        .addPath(
            BezierCurve(
                sampleStartPos,
                Point(15.026, 102.470, Point.CARTESIAN),
                sampleScorePose
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45.0))
        .build()

    var pickupFirstSample: PathChain = builder
        .addPath(
            BezierCurve(
                sampleScorePose,
                Point(16.487, 119.165, Point.CARTESIAN),
                firstSamplePos
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(0.0))
        .build()

    var scoreFirstSample: PathChain = builder
        .addPath(
            BezierLine(
                firstSamplePos,
                sampleScorePose
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45.0))
        .build()

    var pickupSecondSample: PathChain = builder
        .addPath(
            BezierCurve(
                sampleScorePose,
                Point(19.617, 131.896, Point.CARTESIAN),
                secondSamplePos
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(0.0))
        .build()

    var scoreSecondSample: PathChain = builder
        .addPath(
            BezierLine(
                secondSamplePos,
                sampleScorePose
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45.0))
        .build()

    var pickupThirdSample: PathChain = builder
        .addPath(
            BezierCurve(
                sampleScorePose,
                Point(22.957, 119.374, Point.CARTESIAN),
                thirdSamplePos
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(36.0))
        .build()

    var scoreThirdSample: PathChain = builder
        .addPath(
            BezierLine(
                thirdSamplePos,
                sampleScorePose
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(36.0), Math.toRadians(-45.0))
        .build()

    var samplePark: PathChain = builder
        .addPath(
            BezierCurve(
                sampleScorePose,
                Point(66.157, 115.200, Point.CARTESIAN),
                sampleParkPos
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(90.0))
        .build()


    // endregion

    // endregion


}