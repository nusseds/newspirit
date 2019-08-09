package com.nusseds.newspirit

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Intent
import android.os.Bundle
import android.widget.ArrayAdapter
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.activity_main.*
import java.util.*


class MainActivity : AppCompatActivity() {
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

        val pairedDevices = bluetoothAdapter.bondedDevices

        paired_devices.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_list_item_1,
            pairedDevices.map { bluetoothDevice -> bluetoothDevice.name })

        val oppy: List<BluetoothDevice> =
            pairedDevices.filter { bluetoothDevice -> bluetoothDevice.name.equals("oppy") }

        var bluetoothDevice: BluetoothDevice? = null
        if (oppy.size == 1) {
            bluetoothDevice = oppy.get(0)
        }

        val socket: BluetoothSocket? = bluetoothDevice?.createRfcommSocketToServiceRecord(uuid)

        val s = bluetoothDevice?.name ?: "hello"
        println("bluetooth device")
        println(s)
    }
}
