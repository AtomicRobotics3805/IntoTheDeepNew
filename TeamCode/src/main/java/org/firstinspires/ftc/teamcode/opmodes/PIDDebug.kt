package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.components.Components
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadManager
import com.rowanmcalpin.nextftc.hardware.RunToPosition
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension.controller
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension.motor
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension.setPointTolerance
import org.firstinspires.ftc.teamcode.subsystems.motors.Lift
@Config
@TeleOp(name="PID Debug")
class PIDDebug: NextFTCOpMode() {

    override val components = Components()
        .useSubsystems(Extension, Lift)
        .useGamepads()
        .useBulkReading()

    companion object {
        @JvmField
        var inPos = 0.0

        @JvmField
        var outPos = 0.0
    }

    override fun onInit() {
        telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        OpModeData.telemetry = telemetry
        Extension.resetEncoders()
        Lift.resetEncoders()
    }

    override fun onStartButtonPressed() {
        GamepadManager.gamepad1.a.pressedCommand = { RunToPosition(Lift.controller, inPos, Lift.setPointTolerance, Lift) }
        GamepadManager.gamepad1.b.pressedCommand = { RunToPosition(Lift.controller, outPos, Lift.setPointTolerance, Lift) }
        GamepadManager.gamepad1.x.pressedCommand = { RunToPosition(controller, inPos, setPointTolerance, Extension) }
        GamepadManager.gamepad1.y.pressedCommand = { RunToPosition(controller, outPos, setPointTolerance, Extension) }
    }
    override fun onUpdate() {
        telemetry.addData("Lift pos", Lift.motorGroup.currentPosition)
        telemetry.addData("Lift power", Lift.motorGroup.power)
        telemetry.addData("Lift target", Lift.controller.goal.position)
        telemetry.addData("Extension pos", motor.currentPosition)
        telemetry.addData("Extension power", motor.power)
        telemetry.addData("Extension target", controller.goal.position)
        telemetry.update()
    }
}
