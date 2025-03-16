package org.firstinspires.ftc.teamcode.subsystems.servos

import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition

object Pivot: Subsystem() {

    // region Variables

    // region Servo Init

    lateinit var servo: Servo

    @JvmField
    var name = "intake_pivot"

    // endregion

    // region Poses

    @JvmField
    var transferPos = 0.6
    @JvmField
    var intakePos = 0.9

    // endregion

    // endregion

    // region Commands

    val toTransfer: Command
        get() = ServoToPosition(servo, transferPos, this)

    val toIntake: Command
        get() = ServoToPosition(servo, intakePos, this)

    // endregion

    override fun initialize() {
        servo = OpModeData.hardwareMap.get(Servo::class.java, name)
    }
}