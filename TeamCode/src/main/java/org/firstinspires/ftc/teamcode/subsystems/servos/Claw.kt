package org.firstinspires.ftc.teamcode.subsystems.servos

import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition

object Claw: Subsystem() {

    // region Variables

    // region Servo Init

    lateinit var servo: Servo

    @JvmField
    var name = "claw"

    // endregion

    // region Poses

    @JvmField
    var openPos = 0.15
    @JvmField
    var closedPos = 0.35
    @JvmField
    var specimenOpenPos = 0.08

    // endregion

    // endregion

    // region Commands

    val open: Command
        get() = ServoToPosition(servo, openPos, this)

    val close: Command
        get() = ServoToPosition(servo, closedPos, this)

    val specimenOpen: Command
        get() = ServoToPosition(servo, specimenOpenPos, this)

    // endregion

    override fun initialize() {
        servo = OpModeData.hardwareMap.get(Servo::class.java, name)
    }
}