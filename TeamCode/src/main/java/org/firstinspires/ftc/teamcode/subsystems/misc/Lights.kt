package org.firstinspires.ftc.teamcode.subsystems.misc

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import org.firstinspires.ftc.teamcode.other.DotStarBridgedLED

object Lights: Subsystem() {

    // region Variables

    lateinit var ledsLeft: DotStarBridgedLED
    lateinit var ledsRight: DotStarBridgedLED

    var color = 0

    // endregion

    // region Commands

    class display: Command() {
        override val isDone = false

        override fun update() {
            color = when (IntakeSensor.detectedColor) {
                OpModeData.Alliance.RED -> Color.rgb(255, 0, 0)
                OpModeData.Alliance.BLUE -> Color.rgb(0, 0, 255)
                OpModeData.Alliance.NONE -> Color.rgb(255, 255, 0)
                else -> Color.rgb(177, 223, 107)
            }

            for (i in ledsLeft.pixels.indices) {
                // Update individual pixels with their new color.
                ledsLeft.setPixel(i, color)
                ledsRight.setPixel(i, color)
            }
            ledsRight.update()
            ledsLeft.update()
        }
    }

    // endregion

    override fun initialize() {
        ledsLeft.setController(DotStarBridgedLED.Controller.RevExpansionHub)
        ledsRight.setController(DotStarBridgedLED.Controller.RevExpansionHub)
        ledsLeft = OpModeData.hardwareMap.get(DotStarBridgedLED::class.java, "left_leds")
        ledsRight = OpModeData.hardwareMap.get(DotStarBridgedLED::class.java, "right_leds")

        ledsLeft.length = 14
        ledsRight.length = 14

        color = Color.rgb(177, 223, 107)
        for (i in ledsLeft.pixels.indices) {
            // Update individual pixels with their new color.
            ledsLeft.setPixel(i, color)
            ledsRight.setPixel(i, color)
            ledsRight.update()
            ledsLeft.update()
        }

        ledsLeft.maxOutputAmps = 0.8
        ledsRight.maxOutputAmps = 0.8
    }

}