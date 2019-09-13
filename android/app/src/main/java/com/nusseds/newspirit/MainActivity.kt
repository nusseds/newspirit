package com.nusseds.newspirit

import android.content.Intent
import android.os.Bundle
import android.view.Menu
import android.view.MenuItem
import androidx.appcompat.app.AppCompatActivity
import androidx.databinding.ObservableBoolean
import androidx.preference.PreferenceManager
import kotlinx.android.synthetic.main.control_layout.*
import okhttp3.*
import org.json.JSONObject
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt


class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.control_layout)
        setSupportActionBar(findViewById(R.id.toolbar))

        val sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this)

        WEBSOCKET_IP = sharedPrefs.getString("websocket_ip" ,"").toString() // TODO: Handle malformed ip
        WEBSOCKET_PORT = sharedPrefs.getString("websocket_port", "").toString() // TODO: Handle malformed port
        ROS_TOPIC = sharedPrefs.getString("ros_topic", "").toString()

        val wsUrl = "ws://$WEBSOCKET_IP:$WEBSOCKET_PORT"
        val webSocket: WebSocket = ROSconnect(wsUrl)

        joystick.setOnMoveListener({ angle, strength ->
            roverAngle = angle
            roverStrength = strength
            webSocket.send(constructMessage())
        }, 17)
    }

    /**
     * Creates a websocket connection to ROSbridge
     */
    private fun ROSconnect(wsUrl: String) : WebSocket {
        val client = OkHttpClient.Builder()
            .readTimeout(3, TimeUnit.SECONDS)
            .build()
        val request = Request.Builder()
            .url(wsUrl)
            .build()

        val wsListener = ControlWebSocketListener()
        return client.newWebSocket(request, wsListener)
    }

    override fun onCreateOptionsMenu(menu: Menu?): Boolean {
        menuInflater.inflate(R.menu.menu, menu)
        return true;
    }

    override fun onOptionsItemSelected(item: MenuItem) = when (item.itemId) {
        R.id.action_settings -> {
            val intent = Intent(this, SettingsActivity::class.java)
            startActivity(intent)
            true
        }

        else -> {
            // If we got here, the user's action was not recognized.
            // Invoke the superclass to handle it.
            super.onOptionsItemSelected(item)
        }
    }

    private fun constructMessage() : String {
        val angleInRadians = roverAngle * Math.PI / 180
        val steer = (roverStrength * Math.cos(angleInRadians) / 100 * 511 + 511).roundToInt()
        val drive = (roverStrength * Math.sin(angleInRadians) / 100 * 511 + 511).roundToInt()
        val steerDrive = SteerDrive(steer, drive)
        val rosbridgeMsg = JSONObject()
            .put("op", "publish")
            .put("topic", ROS_TOPIC)
            .put("msg", steerDrive.toJSON())
            .toString()
        return rosbridgeMsg
    }

    companion object {
        private var ROS_TOPIC = ""
        private var WEBSOCKET_IP = ""
        private var WEBSOCKET_PORT = ""
        private val ROS_MESSAGE_TYPE = "newspirit/SteerDrive"
        private val NORMAL_CLOSURE_STATUS = 1000

        var roverAngle: Int = 0
        var roverStrength: Int = 0
        val isConnected = ObservableBoolean(false)
    }

    data class SteerDrive(val steer: Int, val drive: Int) {
        fun toJSON() : JSONObject {
            return JSONObject().put("steer", steer)
                .put("drive", drive)
        }
    }

    class ControlWebSocketListener : WebSocketListener() {

        override fun onOpen(webSocket: WebSocket, response: Response) {
            advertise(webSocket, ROS_TOPIC)
            isConnected.set(true)
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

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            webSocket!!.close(NORMAL_CLOSURE_STATUS, null)
            println("Closing websocket")
            unadvertise(webSocket, ROS_TOPIC)
            isConnected.set(false)
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            println("Error: ${t.message}")

        }
    }
}