package com.nusseds.newspirit

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.ArrayAdapter

import kotlinx.android.synthetic.main.activity_connect_device.*

class ConnectDevice : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_connect_device)

        val bluetoothAdapter: BluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
        val pairedDevices: Set<BluetoothDevice> = bluetoothAdapter.bondedDevices

        paired_devices.adapter =
            ArrayAdapter(
                this,
                android.R.layout.simple_list_item_1,
                pairedDevices.map { bluetoothDevice -> bluetoothDevice.name })
    }
}