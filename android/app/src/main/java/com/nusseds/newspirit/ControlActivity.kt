package com.nusseds.newspirit

import android.app.ProgressDialog
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.os.AsyncTask
import android.os.Bundle
import android.util.JsonReader
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.control_layout.*
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString
import org.json.JSONObject
import java.io.IOException
import java.util.*


class ControlActivity : AppCompatActivity() {
    val FREQUENCY = 1 // How many messages per second
    val RPI_IP = "192.168.10.1" // static RPI IP



    companion object {
        private val NORMAL_CLOSURE_STATUS = 1000
        val uuid: UUID = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee") //Standard SerialPortService ID
        var bluetoothSocket: BluetoothSocket? = null
        lateinit var progress: ProgressDialog
        lateinit var deviceAddress: String
        var isConnected: Boolean = false
        var roverAngle: Int = 0
        var roverStrength: Int = 0
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.control_layout)
        deviceAddress = intent.getStringExtra("deviceAddress")

        ConnectToDevice(this).execute()

        values_isConnected.apply {
            text = if (isConnected) "Connected" else "Disconnected"
        }

        joystick.setOnMoveListener({ angle, strength ->
            roverAngle = angle
            roverStrength = strength
            values_angle.text = roverAngle.toString()
            values_strength.text = roverStrength.toString()
        }, 17)

        // Periodically send bluetooth messages
        val timer = Timer()
        val context = this

        val task = object: TimerTask() {
            override fun run() {
                SendMessage(context).execute()
            }
        }

        timer.schedule(task, Math.round(1000.0 / FREQUENCY), Math.round(1000.0 / FREQUENCY))

        disconnect_button.setOnClickListener { _ ->
            disconnect()
        }
    }

    private class SendMessage(c: Context): AsyncTask<Void, Void, String>() {
        private fun constructMessage() : String {
            return JSONObject()
                .put("rover_control", JSONObject()
                    .put("angle", roverAngle)
                    .put("strength", roverStrength))
                .toString()
        }

        private fun sendCommand(input: String) {
            if (bluetoothSocket != null) {
                if (!bluetoothSocket!!.isConnected) {
                    bluetoothSocket!!.connect()
                }
                try {
                    bluetoothSocket!!.outputStream.write(input.toByteArray())
                } catch (e: IOException) {
                    e.printStackTrace()
                }
            }
        }

        override fun doInBackground(vararg p0: Void?): String? {
            if (bluetoothSocket != null && bluetoothSocket!!.isConnected) {
                sendCommand(constructMessage())
            }
            return null
        }
    }

    private fun disconnect() {
        if (bluetoothSocket != null && bluetoothSocket!!.isConnected) {
            try {
                bluetoothSocket!!.close()
                bluetoothSocket = null
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
        finish()
    }

    private class ConnectToDevice(c: Context) : AsyncTask<Void, Void, String>() {
        private var connectSuccess: Boolean = true
        private val context: Context = c

        override fun onPreExecute() {
            super.onPreExecute()
            progress = ProgressDialog.show(context, "Connecting...", "please wait")
        }

        override fun doInBackground(vararg p0: Void?): String? {
            try {
                if (bluetoothSocket == null || !bluetoothSocket!!.isConnected) {
                    val bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
                    val device: BluetoothDevice = bluetoothAdapter.getRemoteDevice(deviceAddress)
                    bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
                    bluetoothAdapter.cancelDiscovery()
                    bluetoothSocket!!.connect()
                    connectSuccess = true
                }
            } catch (e: IOException) {
                connectSuccess = false
                e.printStackTrace()
            }
            return null
        }

        override fun onPostExecute(result: String?) {
            super.onPostExecute(result)
            if (!connectSuccess) {
                Toast.makeText(context, "Can't connect to rover.", Toast.LENGTH_SHORT).show()
            } else {
                isConnected = true
            }
            progress.dismiss()
        }
    }
}
