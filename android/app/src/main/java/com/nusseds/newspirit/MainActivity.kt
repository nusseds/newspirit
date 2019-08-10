package com.nusseds.newspirit

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.activity_main.*
import java.util.*


class MainActivity : AppCompatActivity() {
    val CONNECT_DEVICE_REQUEST_CODE = 0
    val uuid: UUID = UUID.fromString("94f39d29-7d6d-437d-973b-fba39e49d4ee") //Standard SerialPortService ID

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Enable Bluetooth if not already enabled
        val bluetoothAdapter: BluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        if (bluetoothAdapter == null) {
            AlertDialog.Builder(this)
                .setTitle("Incompatible")
                .setMessage("Your device does not support Bluetooth")
                .setPositiveButton("Exit") { _, _ -> System.exit(0) }
                .setIcon(android.R.drawable.ic_dialog_alert)
                .show()
        }

        if (!bluetoothAdapter.isEnabled) {
            val enableBluetooth: Intent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBluetooth, 0)
        }

        connect_button.setOnClickListener { v ->
            println("Launching Connect Device activity")
            val intent = Intent(this, ConnectDevice::class.java)
            startActivityForResult(intent, CONNECT_DEVICE_REQUEST_CODE)
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)

        if (requestCode == CONNECT_DEVICE_REQUEST_CODE && resultCode == RESULT_OK) {
            val device : BluetoothDevice? = data?.extras!!.getParcelable("btDevice")
            Toast.makeText(this, "Connecting to " + device?.name, Toast.LENGTH_SHORT).show()
            connect_button.text = "Connecting..."

            //TODO: Establish connection to bluetooth device
        }
    }
}
