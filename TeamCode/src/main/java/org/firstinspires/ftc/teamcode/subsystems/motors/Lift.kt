package org.firstinspires.ftc.teamcode.subsystems.motors

import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition

object Lift: Subsystem() {

    // region Variables

    // region Motor Init

    lateinit var rightMotor: MotorEx
    lateinit var leftMotor: MotorEx

    lateinit var motorGroup: MotorGroup

    @JvmField
    var rightMotorName = "lift"
    @JvmField
    var leftMotorName = "lift2"

    // endregion

    // region PID

    @JvmField
    var kP = 0.005 // TODO: Tune

    @JvmField
    var kI = 0.005 // TODO: Tune

    @JvmField
    var kD = 0.005 // TODO: Tune

    @JvmField
    var kF = 0.13 // TODO: Tune

    @JvmField
    var setPointTolerance = 10.0 // TODO: Tune

    var controller = PIDFController(kP, kI, kD, { kF }, setPointTolerance)

    // endregion

    // region Poses

    @JvmField
    var autoTransferPos = 100.0
    @JvmField
    var intakePos = 85.0
    @JvmField
    var specimenPickupPos = 180.0
    @JvmField
    var highPos = 3700.0
    @JvmField
    var slightlyHighPos = 400.0
    @JvmField
    var specimenScorePos = 680.0
    @JvmField
    var specimenAutonomousScorePos = 277.0
    @JvmField
    var hangPos = 2000.0

    // endregion

    // endregion

    // region Commands

    override val defaultCommand: Command
        get() = HoldPosition(motorGroup, controller, this)

    val resetEncoders: Command
        get() = ResetEncoder(motorGroup.leader, this)

    val toIntake: Command
        get() = RunToPosition(motorGroup, intakePos, controller, this)

    val toSpecimenPickup: Command
        get() = RunToPosition(motorGroup, specimenPickupPos, controller, this)

    val toHigh: Command
        get() = RunToPosition(motorGroup, highPos, controller, this)

    val toSlightlyHigh: Command
        get() = RunToPosition(motorGroup, slightlyHighPos, controller, this)

    val toSpecimenScore: Command
        get() = RunToPosition(motorGroup, specimenScorePos, controller, this)

    val toAutonomousSpecScore: Command
        get() = RunToPosition(motorGroup, specimenAutonomousScorePos, controller, this)

    val toHang: Command
        get() = RunToPosition(motorGroup, hangPos, controller, this)

    val toAutoTransferPos: Command
        get() = RunToPosition(motorGroup, autoTransferPos, controller, this)

    val zero: Command
        get() = RunToPosition(motorGroup, 0.0, controller, this)

    // endregion

    override fun periodic() {
        controller = PIDFController(kP, kI, kD, { kF }, setPointTolerance) // Update PID
    }

    override fun initialize() {
        rightMotor = MotorEx(rightMotorName).reverse()
        leftMotor = MotorEx(leftMotorName)
        motorGroup = MotorGroup(rightMotor, leftMotor)
    }
}