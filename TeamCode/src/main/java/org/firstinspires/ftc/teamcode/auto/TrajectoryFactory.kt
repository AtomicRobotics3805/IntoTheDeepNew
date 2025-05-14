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


    var line1: PathChain = builder
        .addPath(
            BezierCurve(
                Point(0.000, 0.000, Point.CARTESIAN),
                Point(26.704, 46.919, Point.CARTESIAN),
                Point(23.459, 10.482, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(0.0))
        .build()

    var line2: PathChain = builder
        .addPath(
            BezierLine(
                Point(23.459, 10.482, Point.CARTESIAN),
                Point(10.981, 14.475, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(56.0))
        .build()

    var line3: PathChain = builder
        .addPath(
            BezierLine(
                Point(10.981, 14.475, Point.CARTESIAN),
                Point(8.236, 23.958, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(56.0), Math.toRadians(180.0))
        .build()

    var line4: PathChain = builder
        .addPath(
            BezierLine(
                Point(8.236, 23.958, Point.CARTESIAN),
                Point(11.730, 24.707, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180.0))
        .build()

    var line5: PathChain = builder
        .addPath(
            BezierLine(
                Point(11.730, 24.707, Point.CARTESIAN),
                Point(13.726, 18.218, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180.0))
        .build()

    var line6: PathChain = builder
        .addPath(
            BezierLine(
                Point(13.726, 18.218, Point.CARTESIAN),
                Point(8.236, 20.714, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(0.0))
        .build()

    var line7: PathChain = builder
        .addPath(
            BezierLine(
                Point(8.236, 20.714, Point.CARTESIAN),
                Point(0.0, 0.0, Point.CARTESIAN)
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