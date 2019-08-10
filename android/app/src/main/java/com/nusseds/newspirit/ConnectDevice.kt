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
            parent, view, position, id ->
            Toast.makeText(this, "Clicked item: " + position, Toast.LENGTH_SHORT).show()
            val device: BluetoothDevice = pairedDevices.get(position)
            val intent = Intent()
                .putExtra("btDevice", device)
            setResult(RESULT_OK, intent)
            finish()
        }
    }
}