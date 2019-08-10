package com.nusseds.newspirit

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Toast

import kotlinx.android.synthetic.main.activity_connect_device.*
import org.jetbrains.anko.startActivity
import org.jetbrains.anko.toast

class ConnectDevice : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_connect_device)

        val bluetoothAdapter: BluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
        val pairedDevices: List<BluetoothDevice> = bluetoothAdapter.bondedDevices.toList()

        paired_devices.adapter =
            ArrayAdapter(
                this,
                android.R.layout.simple_list_item_1,
                pairedDevices.map { bluetoothDevice -> bluetoothDevice.name })

        paired_devices.onItemClickListener = AdapterView.OnItemClickListener {
            _, _, position, _ ->
            val device: BluetoothDevice = pairedDevices[position]
            toast("Connecting to " + device.name)
            startActivity<ControlActivity>("deviceAddress" to device.address)
        }
    }
}