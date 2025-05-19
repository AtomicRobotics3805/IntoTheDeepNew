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
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension.transferPos

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
    var setPointTolerance = 100.0

    @JvmField
    var coefficients = PIDCoefficients(0.003, 0.0, 0.00008)

    @JvmField
    var secondCoefficients = PIDCoefficients(0.03, 0.0, 0.0)

    @JvmField
    var ffParameters = GravityFeedforwardParameters(0.17)


    @JvmField
    var secondffParameters = GravityFeedforwardParameters(0.07)

    @JvmField
    var interpParameters = FirstOrderEMAParameters(0.2)


    @JvmField
    var secondInterpParameters = FirstOrderEMAParameters(0.2)

    @JvmField
    var switch = KineticState(100.0)

    val controller = ControlSystem().posPid(coefficients).elevatorFF(ffParameters)
        .emaInterpolator(interpParameters).build()

    val secondController = ControlSystem().posPid(secondCoefficients).emaInterpolator(secondInterpParameters).elevatorFF(
        secondffParameters).build()

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
    var hangPos = 2030.0

    // endregion

    // endregion

    // region Commands

    val resetEncoders: Command
        get() = ResetEncoder(motorGroup.leader, this)

    val toIntake: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, intakePos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toSpecimenPickup: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, specimenPickupPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toHigh: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, highPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toSlightlyHigh: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, slightlyHighPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toSpecimenScore: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, specimenScorePos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toAutonomousSpecScore: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, specimenAutonomousScorePos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toHang: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, hangPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val toAutoTransferPos: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, autoTransferPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val zero: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, 0.0,
            switch,
            setPointTolerance,
            setOf(this)
        )

    val hang: Command
        get() = dualControllerRunToPosition(
            controller,
            secondController, slightlyHighPos,
            switch,
            setPointTolerance,
            setOf(this)
        )

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