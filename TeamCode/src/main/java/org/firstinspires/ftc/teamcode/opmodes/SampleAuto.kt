
package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.components.Components
import com.rowanmcalpin.nextftc.pedro.PedroComponent
import com.rowanmcalpin.nextftc.pedro.PedroData
import org.firstinspires.ftc.teamcode.auto.FConstants
import org.firstinspires.ftc.teamcode.auto.LConstants
import org.firstinspires.ftc.teamcode.routines.SampleRoutines
import org.firstinspires.ftc.teamcode.subsystems.misc.IntakeSensor
import org.firstinspires.ftc.teamcode.subsystems.misc.Lights
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension
import org.firstinspires.ftc.teamcode.subsystems.motors.Lift
import org.firstinspires.ftc.teamcode.subsystems.servos.Arm
import org.firstinspires.ftc.teamcode.subsystems.servos.Claw
import org.firstinspires.ftc.teamcode.subsystems.servos.Intake
import org.firstinspires.ftc.teamcode.subsystems.servos.Pivot

@Autonomous(name = "Sample Auto")
class SampleAuto: NextFTCOpMode() {

    override val components = Components()
        .useSubsystems(Claw, Intake, Arm, Extension, Pivot, IntakeSensor, Lift, Lights)
        .use(PedroComponent(FConstants::class.java, LConstants::class.java))
        .useBulkReading()

    override fun onInit() {

        PedroData.follower!!.poseUpdater.resetIMU()

        IntakeSensor.detect()()
        Lights.display()()

        Extension.resetEncoders()
        Lift.resetEncoders()
        Arm.toDown()
        Pivot.toTransfer()
        Claw.close()

        OpModeData.telemetry = telemetry
        Extension.motor.currentPosition = -35.0
    }


    override fun onStartButtonPressed() {
        SampleRoutines.sampleAuto()
    }
}
