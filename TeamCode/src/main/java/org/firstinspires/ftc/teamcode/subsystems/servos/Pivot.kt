package org.firstinspires.ftc.teamcode.subsystems.servos

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition

@Config
object Pivot : Subsystem() {

    // region Variables

    // region Servo Init

    lateinit var servo: Servo

    @JvmField
    var name = "intake_pivot"

    // endregion

    // region Poses

    @JvmField
    var transferPos = 0.7

    @JvmField
    var intakePos = 0.85

    // endregion

    // endregion

    // region Commands

    val toTransfer: Command
        get() = ServoToPosition(servo, transferPos, this)

    val toIntake: Command
        get() = ServoToPosition(servo, intakePos, this)

    // endregion

    override fun initialize() {
        servo = OpModeData.hardwareMap!!.get(Servo::class.java, name)
    }
}