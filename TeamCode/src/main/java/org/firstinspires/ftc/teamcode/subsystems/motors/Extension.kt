package org.firstinspires.ftc.teamcode.subsystems.motors

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition

object Extension: Subsystem() {

    // region Variables

    // region Motor Init

    lateinit var motor: MotorEx

    lateinit var motorGroup: MotorGroup

    @JvmField
    var motorName = "intake_extension"

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
    var transferPos = 100.0
    @JvmField
    var autoTransferPos = 70.0
    @JvmField
    var outPos = 900.0
    @JvmField
    var slightlyOutPos = 700.0
    @JvmField
    var middlePos = 600.0
    @JvmField
    var autoOutPos = 600.0

    // endregion

    // endregion

    // region Commands

    override val defaultCommand: Command
        get() = HoldPosition(motorGroup, controller, this)

    val resetEncoders: Command
        get() = ResetEncoder(motorGroup.leader, this)

    val toTransfer: Command
        get() = RunToPosition(motor, transferPos, controller, this)

    val toOut: Command
        get() = RunToPosition(motor, outPos, controller, this)

    val toSlightlyOut: Command
        get() = RunToPosition(motor, slightlyOutPos, controller, this)

    val toMiddlePos: Command
        get() = RunToPosition(motor, middlePos, controller, this)

    val toIntakeAuto: Command
        get() = RunToPosition(motor, autoOutPos, controller, this)

    val toAutoTransfer: Command
        get() = RunToPosition(motor, autoTransferPos, controller, this)

    val toFullIn: Command
        get() = RunToPosition(motor, 0.0, controller, this)

    // endregion

    override fun periodic() {
        controller = PIDFController(kP, kI, kD, { kF }, setPointTolerance) // Update PID
    }

    override fun initialize() {
        motor = MotorEx(motorName)
        motor.direction = DcMotorSimple.Direction.REVERSE
        controller.setPointTolerance = 10.0
    }
}