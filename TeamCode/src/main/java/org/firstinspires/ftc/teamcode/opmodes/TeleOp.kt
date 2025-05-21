package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.localization.PoseUpdater
import com.pedropathing.localization.constants.TwoWheelConstants
import com.pedropathing.util.Constants
import com.pedropathing.util.DashboardPoseTracker
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.components.Components
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadManager
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.pedro.PedroData.follower
import dev.nextftc.nextcontrol.KineticState
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.MecanumDriverControlledFixed
import org.firstinspires.ftc.teamcode.auto.FConstants
import org.firstinspires.ftc.teamcode.auto.LConstants
import org.firstinspires.ftc.teamcode.routines.MechanismRoutines
import org.firstinspires.ftc.teamcode.subsystems.misc.IntakeSensor
import org.firstinspires.ftc.teamcode.subsystems.misc.Lights
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension
import org.firstinspires.ftc.teamcode.subsystems.motors.Lift
import org.firstinspires.ftc.teamcode.subsystems.servos.Arm
import org.firstinspires.ftc.teamcode.subsystems.servos.Claw
import org.firstinspires.ftc.teamcode.subsystems.servos.Intake
import org.firstinspires.ftc.teamcode.subsystems.servos.Pivot

@TeleOp(name = "TeleOp")
class TeleOp : NextFTCOpMode() {

    override val components = Components()
        .useSubsystems(Claw, Intake, Arm, Extension, Pivot, IntakeSensor, Lift, Lights)
        .useGamepads()
        .useBulkReading()




    private lateinit var leftFront: MotorEx
    private lateinit var leftRear: MotorEx
    private lateinit var rightFront: MotorEx
    private lateinit var rightRear: MotorEx

    lateinit var motors: Array<MotorEx>

    lateinit var imu: IMU

    lateinit var driverControlled: MecanumDriverControlledFixed


    private val fConstants = FConstants()
    private val lConstants = LConstants()


    override fun onInit() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)

        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        OpModeData.telemetry = telemetry


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
        // Lights.display()()

        Arm.toDown()
        Pivot.toTransfer()

        Lift.controller.goal = KineticState(0.0)
        Lift.secondController.goal = KineticState(0.0)
        Extension.controller.goal = KineticState(0.0)
//        Extension.secondController.goal = KineticState(0.0)
        Extension.resetEncoders()
        Lift.resetEncoders()

        OpModeData.telemetry = telemetry
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlledFixed(motors, GamepadManager.gamepad1, false, imu)
        driverControlled()
        driverControlled.orientation = Math.toRadians(90.0)
        registerControls()
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


        telemetry.update()
    }

    private fun registerControls() {


        GamepadManager.gamepad2.rightBumper.pressedCommand = { Intake.eject }
        GamepadManager.gamepad2.rightBumper.releasedCommand = { Intake.intake }

        GamepadManager.gamepad1.leftTrigger.pressedCommand = {
            InstantCommand {
                driverControlled.scalar = 0.5
            }
        }
        GamepadManager.gamepad1.leftTrigger.releasedCommand = {
            InstantCommand {
                driverControlled.scalar = 1.0
            }
        }

        GamepadManager.gamepad1.x.pressedCommand = {
            InstantCommand {
                driverControlled.orientation = 0.0
            }
        }

        GamepadManager.gamepad1.rightBumper.pressedCommand = { MechanismRoutines.outToIntake }
        GamepadManager.gamepad1.rightBumper.releasedCommand = { MechanismRoutines.transfer }

        GamepadManager.gamepad1.leftBumper.pressedCommand = { MechanismRoutines.nearIntake }
        GamepadManager.gamepad1.leftBumper.releasedCommand = { MechanismRoutines.transfer }

        GamepadManager.gamepad1.rightTrigger.pressedCommand = { MechanismRoutines.scoreAndReset }

        GamepadManager.gamepad2.a.pressedCommand = { Claw.open }
        GamepadManager.gamepad2.b.pressedCommand = { MechanismRoutines.jostle }


        GamepadManager.gamepad1.dpadUp.pressedCommand = { MechanismRoutines.toHang }
        GamepadManager.gamepad1.dpadDown.pressedCommand = { MechanismRoutines.hang }

        GamepadManager.gamepad1.y.pressedCommand = { MechanismRoutines.fullReset }

        GamepadManager.gamepad2.y.pressedCommand = { MechanismRoutines.specimenPickup }
        GamepadManager.gamepad2.leftTrigger.pressedCommand = { MechanismRoutines.specimenScore }

        GamepadManager.gamepad2.dpadUp.pressedCommand = { MechanismRoutines.scoreSample }
        GamepadManager.gamepad2.dpadRight.pressedCommand = { MechanismRoutines.scoreSampleLow }

    }
}