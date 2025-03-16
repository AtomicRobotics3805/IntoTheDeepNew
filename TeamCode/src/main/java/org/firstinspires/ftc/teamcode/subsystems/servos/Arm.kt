package org.firstinspires.ftc.teamcode.subsystems.servos

import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition

object Arm: Subsystem() {

    // region Variables

    // region Servo Init

    lateinit var servo: Servo

    @JvmField
    var name = "arm"

    // endregion

    // region Poses

    @JvmField
    var upPos = 0.7
    @JvmField
    var downPos = 0.05
    @JvmField
    var specimenPose = 0.97

    // endregion

    // endregion

    // region Commands

    val toUp: Command
        get() = ServoToPosition(servo, upPos, this)

    val toDown: Command
        get() = ServoToPosition(servo, downPos, this)

    val toSpecimenPickup: Command
        get() = ServoToPosition(servo, specimenPose, this)

    // endregion

    override fun initialize() {
        servo = OpModeData.hardwareMap.get(Servo::class.java, name)
    }
}