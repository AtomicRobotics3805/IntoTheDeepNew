package org.firstinspires.ftc.teamcode.subsystems.servos

import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition

object Intake : Subsystem() {

    // region Variables

    // region Servo Init

    lateinit var leftServo: Servo
    lateinit var rightServo: Servo

    @JvmField
    var leftName = "intake_left"

    @JvmField
    var rightName = "intake_right"

    // endregion

    // endregion

    // region Commands

    val intake: Command
        get() = MultipleServosToPosition(listOf(leftServo, rightServo), 1.0, this)

    val eject: Command
        get() = MultipleServosToPosition(listOf(leftServo, rightServo), 0.0, this)

    val stop: Command
        get() = MultipleServosToPosition(listOf(leftServo, rightServo), 0.5, this)

    val slowIntake: Command
        get() = MultipleServosToPosition(listOf(leftServo, rightServo), 0.6, this)

    // endregion

    override fun initialize() {
        leftServo = OpModeData.hardwareMap!!.get(Servo::class.java, leftName)
        rightServo = OpModeData.hardwareMap!!.get(Servo::class.java, rightName)
        rightServo.direction = Servo.Direction.REVERSE
    }
}