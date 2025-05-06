package org.firstinspires.ftc.teamcode.subsystems.motors

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder
import dev.nextftc.nextcontrol.ControlSystem
import com.rowanmcalpin.nextftc.hardware.RunToPosition
import dev.nextftc.nextcontrol.KineticState
import dev.nextftc.nextcontrol.feedback.PIDCoefficients
import dev.nextftc.nextcontrol.feedforward.GravityFeedforwardParameters
import dev.nextftc.nextcontrol.interpolators.FirstOrderEMAParameters

@Config
object Extension : Subsystem() {

    // region Variables

    // region Motor Init

    lateinit var motor: MotorEx


    @JvmField
    var motorName = "intake_extension"

    // endregion

    // region PID

    @JvmField
    var setPointTolerance = 30.0

    @JvmField
    var coefficients = PIDCoefficients(0.005, 0.0, 0.0)

    @JvmField
    var ffParameters = GravityFeedforwardParameters(0.15)


    @JvmField
    var interpParameters = FirstOrderEMAParameters(0.1)

    val controller = ControlSystem().posPid(coefficients).emaInterpolator(interpParameters)
        .elevatorFF(ffParameters).build()

    // endregion

    // region Poses

    @JvmField
    var transferPos = 170.0

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

    val resetEncoders: Command
        get() = ResetEncoder(motor, this)

    val toTransfer: Command
        get() = RunToPosition(controller, transferPos, setPointTolerance, this)

    val toOut: Command
        get() = RunToPosition(controller, outPos, setPointTolerance, this)

    val toSlightlyOut: Command
        get() = RunToPosition(controller, slightlyOutPos, setPointTolerance, this)

    val toMiddlePos: Command
        get() = RunToPosition(controller, middlePos, setPointTolerance, this)

    val toIntakeAuto: Command
        get() = RunToPosition(controller, autoOutPos, setPointTolerance, this)

    val toAutoTransfer: Command
        get() = RunToPosition(controller, autoTransferPos, setPointTolerance, this)

    val toFullIn: Command
        get() = RunToPosition(controller, 0.0, setPointTolerance, this)

    // endregion

    override fun periodic() {
        motor.power = controller.calculate(KineticState(motor.currentPosition, motor.velocity))
    }

    override fun initialize() {
        motor = MotorEx(motorName)
        motor.direction = DcMotorSimple.Direction.FORWARD
    }
}