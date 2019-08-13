# Setting up the Raspberry Pi

We use the raspberry pi as the "brain" of the rover. It reads signals
from an Android app (such as rover movement commands), and then
rebroadcasts these commands via ROS to the respective
micro-controllers. These signals are received via Bluetooth.

## Setting up Bluetooth

Because of updates to Bluez to use the latest DBus api, we need to run
Bluez in compatibility mode. To do so, we need to modify the systemctl
service that runs the bluez daemon:

`sudo vim /etc/systemd/system/dbus-org.bluez.service` and add the `--compat`
flag to the bluez command

```text
ExecStart=/usr/lib/bluetooth/bluetoothd --compat
```

Then run `sudo systemd daemon-reload` and `sudo systemctl restart
bluetooth`.

Once that is done, the socket `/var/run/sdp` would then be open. Let's
be lazy and enable read-write-execute on for everyone on this socket:

`sudo chmod 777 /var/run/sdp`

And we're ready to accept connections to this Bluetooth port.
