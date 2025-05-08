package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.Pose
import com.pedropathing.localization.constants.TwoWheelConstants
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.components.Components
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.pedro.PedroData.follower
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.MecanumDriverControlledFixed
import org.firstinspires.ftc.teamcode.auto.FConstants
import org.firstinspires.ftc.teamcode.auto.LConstants
import org.firstinspires.ftc.teamcode.routines.SampleRoutines
import org.firstinspires.ftc.teamcode.subsystems.misc.IntakeSensor
import org.firstinspires.ftc.teamcode.subsystems.misc.Lights
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension
import org.firstinspires.ftc.teamcode.subsystems.motors.Lift
import org.firstinspires.ftc.teamcode.subsystems.servos.Arm
import org.firstinspires.ftc.teamcode.subsystems.servos.Claw
import org.firstinspires.ftc.teamcode.subsystems.servos.Intake
import org.firstinspires.ftc.teamcode.subsystems.servos.Pivot

@Autonomous(name = "Sample Auto Slow")
class SampleAutoSlow : NextFTCOpMode() {

    override val components = Components()
        .useSubsystems(Claw, Intake, Arm, Extension, Pivot, IntakeSensor, Lift, Lights)
        .use(Pedro(FConstants::class.java, LConstants::class.java))
        .useBulkReading()




    private lateinit var leftFront: MotorEx
    private lateinit var leftRear: MotorEx
    private lateinit var rightFront: MotorEx
    private lateinit var rightRear: MotorEx

    lateinit var motors: Array<MotorEx>

    lateinit var imu: IMU

    private val fConstants = FConstants()
    private val lConstants = LConstants()

    lateinit var driverControlled: MecanumDriverControlledFixed


    override fun onInit() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        OpModeData.telemetry = telemetry
        follower!!.setStartingPose(Pose(9.0, 105.0))

        leftFront = MotorEx(FollowerConstants.leftFrontMotorName)
        leftRear = MotorEx(FollowerConstants.leftRearMotorName)
        rightRear = MotorEx(FollowerConstants.rightRearMotorName)
        rightFront = MotorEx(FollowerConstants.rightFrontMotorName)

        motors = arrayOf(leftFront, rightFront, leftRear, rightRear)

        leftFront.direction = FollowerConstants.leftFrontMotorDirection
        leftRear.direction = FollowerConstants.leftRearMotorDirection
        rightFront.direction = FollowerConstants.rightFrontMotorDirection
        rightRear.direction = FollowerConstants.rightRearMotorDirection

        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.initialize(IMU.Parameters(TwoWheelConstants.IMU_Orientation))
        imu.resetYaw()

        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        IntakeSensor.detect()()
        Lights.display()()

        Arm.toDown()
        Pivot.toTransfer()


        Extension.resetEncoders()
        Lift.resetEncoders()

        OpModeData.telemetry = telemetry
    }

    override fun onStartButtonPressed() {
        SampleRoutines.sampleAutoSlow()
    }

    override fun onUpdate() {
        telemetry.addData("Color", IntakeSensor.detectedColor)
        telemetry.addData("Distance", IntakeSensor.sensor.getDistance(DistanceUnit.CM))
        telemetry.addData("Lift power left", Lift.leftMotor.power)
        telemetry.addData("Lift power right", Lift.rightMotor.power)
        telemetry.addData("Lift target position", Lift.controller.reference.position)
        telemetry.addData("Lift current position", Lift.motorGroup.currentPosition)
        telemetry.addData("Extension Pos", Extension.motor.currentPosition)
        telemetry.addData("Extension Target", Extension.controller.reference.position)
        follower!!.telemetryDebug(telemetry)

        telemetry.update()
    }

}

