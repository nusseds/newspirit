package com.nusseds.newspirit

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.control_layout.*
import okhttp3.*
import okio.ByteString
import org.json.JSONObject
import java.util.*
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt


class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.control_layout)

        val client = OkHttpClient.Builder()
            .readTimeout(3, TimeUnit.SECONDS)
            .build()
        val request = Request.Builder()
            .url("ws://$RPI_STATIC_IP:$RPI_WS_PORT")
            .build()

        val wsListener = ControlWebSocketListener()
        val webSocket: WebSocket = client.newWebSocket(request, wsListener)

        joystick.setOnMoveListener({ angle, strength ->
            roverAngle = angle
            roverStrength = strength
            values_angle.text = roverAngle.toString()
            values_strength.text = roverStrength.toString()
            println(constructMessage())
            webSocket.send(constructMessage())
        }, 17)
    }


    private fun constructMessage() : String {
//        val angleInRadians = roverAngle * Math.PI / 180
//        val steer = (roverStrength * Math.cos(angleInRadians) / 100 * 511 + 511).roundToInt()
//        val drive = (roverStrength * Math.sin(angleInRadians) / 100 * 511 + 511).roundToInt()
        val msg = JSONObject()
            .put("angle", roverAngle)
            .put("strength", roverStrength)
            .toString()

        val rosbridgeMsg = JSONObject()
            .put("op", "publish")
            .put("topic", ROS_TOPIC_NAME)
            .put("msg", JSONObject()
                .put("data", msg))
            .toString()
        return rosbridgeMsg
    }

    companion object {
        private val RPI_STATIC_IP = "192.168.10.1"
        private val RPI_WS_PORT = "9090"
        private val ROS_MESSAGE_TYPE = "SteerDrive"
//        private val ROS_TOPIC_NAME = "/newspirit/cmd/steerDrive"
        private val ROS_TOPIC_NAME = "/newspirit/remote"
        private val NORMAL_CLOSURE_STATUS = 1000

        var roverAngle: Int = 0
        var roverStrength: Int = 0
    }

    class ControlWebSocketListener : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            println("Connected to Oppy")
            advertise(webSocket, ROS_TOPIC_NAME)
        }

        private fun advertise(websocket: WebSocket, topicName: String) {
            val payload = JSONObject()
                .put("op", "advertise")
                .put("topic", topicName)
                .put("type", ROS_MESSAGE_TYPE)
                .toString()
            websocket.send(payload)
        }

        private fun unadvertise(websocket: WebSocket, topicName: String) {
            val payload = JSONObject()
                .put("op", "unadvertise")
                .put("topic", topicName)
                .toString()
            websocket.send(payload)
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            println("Receiving: $text")
        }

        override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
            println("Receiving bytes: $bytes")
        }

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            webSocket!!.close(NORMAL_CLOSURE_STATUS, null)
            println("Closing websocket")
            unadvertise(webSocket, ROS_TOPIC_NAME)
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            println("Error: ${t.message}")
        }
    }
}
