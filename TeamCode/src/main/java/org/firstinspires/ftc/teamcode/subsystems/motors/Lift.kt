package org.firstinspires.ftc.teamcode.subsystems.motors

import com.acmerobotics.dashboard.config.Config
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder
import com.rowanmcalpin.nextftc.hardware.RunToPosition
import dev.nextftc.nextcontrol.ControlSystem
import dev.nextftc.nextcontrol.KineticState
import dev.nextftc.nextcontrol.feedback.PIDCoefficients
import dev.nextftc.nextcontrol.feedforward.GravityFeedforwardParameters
import dev.nextftc.nextcontrol.interpolators.FirstOrderEMAParameters

@Config
object Lift : Subsystem() {

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
    var setPointTolerance = 30.0

    @JvmField
    var coefficients = PIDCoefficients(0.003, 0.0, 0.00008)

    @JvmField
    var ffParameters = GravityFeedforwardParameters(0.17)

    @JvmField
    var interpParameters = FirstOrderEMAParameters(0.2)

    val controller = ControlSystem().posPid(coefficients).elevatorFF(ffParameters)
        .emaInterpolator(interpParameters).build()

    // endregion

    // region Poses

    @JvmField
    var autoTransferPos = 100.0

    @JvmField
    var intakePos = 85.0

    @JvmField
    var specimenPickupPos = 200.0

    @JvmField
    var highPos = 3700.0

    @JvmField
    var slightlyHighPos = 310.0

    @JvmField
    var specimenScorePos = 800.0

    @JvmField
    var specimenAutonomousScorePos = 277.0

    @JvmField
    var hangPos = 2010.0

    // endregion

    // endregion

    // region Commands

    val resetEncoders: Command
        get() = ResetEncoder(motorGroup.leader, this)

    val toIntake: Command
        get() = RunToPosition(controller, intakePos, setPointTolerance, this)

    val toSpecimenPickup: Command
        get() = RunToPosition(controller, specimenPickupPos, setPointTolerance, this)

    val toHigh: Command
        get() = RunToPosition(controller, highPos, setPointTolerance, this)

    val toSlightlyHigh: Command
        get() = RunToPosition(controller, slightlyHighPos, setPointTolerance, this)

    val toSpecimenScore: Command
        get() = RunToPosition(controller, specimenScorePos, setPointTolerance, this)

    val toAutonomousSpecScore: Command
        get() = RunToPosition(controller, specimenAutonomousScorePos, setPointTolerance, this)

    val toHang: Command
        get() = RunToPosition(controller, hangPos, setPointTolerance, this)

    val toAutoTransferPos: Command
        get() = RunToPosition(controller, autoTransferPos, setPointTolerance, this)

    val zero: Command
        get() = RunToPosition(controller, 0.0, setPointTolerance, this)

    val hang: Command
        get() = RunToPosition(controller, slightlyHighPos, setPointTolerance, this)

    // endregion

    override fun periodic() {
        motorGroup.power = controller.calculate(KineticState(motorGroup.currentPosition, motorGroup.velocity))
    }

    override fun initialize() {
        rightMotor = MotorEx(rightMotorName).reverse()
        leftMotor = MotorEx(leftMotorName)
        motorGroup = MotorGroup(rightMotor, leftMotor)
    }
}