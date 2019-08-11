package com.nusseds.newspirit

import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.os.Bundle
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.activity_main.*
import org.jetbrains.anko.startActivity
import org.jetbrains.anko.toast


class MainActivity : AppCompatActivity() {
    var bluetoothAdapter: BluetoothAdapter? = null

    private val REQUEST_ENABLE_BLUETOOTH = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Enable Bluetooth if not already enabled
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        if (bluetoothAdapter == null) {
            AlertDialog.Builder(this)
                .setTitle("Incompatible")
                .setMessage("Your device does not support Bluetooth")
                .setPositiveButton("Exit") { _, _ -> System.exit(0) }
                .setIcon(android.R.drawable.ic_dialog_alert)
                .show()
        }

        if (!bluetoothAdapter!!.isEnabled) {
            val enableBluetoothIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBluetoothIntent, REQUEST_ENABLE_BLUETOOTH)
        }

        connect_button.setOnClickListener { v ->
            startActivity<ConnectDevice>()
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == REQUEST_ENABLE_BLUETOOTH && resultCode == RESULT_OK) {
            if (bluetoothAdapter!!.isEnabled) {
                toast("Bluetooth enabled")
            } else {
                toast("Bluetooth disabled")
            }
        }
    }
}
