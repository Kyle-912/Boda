import 'package:boda_controller/src/ble/ble_device_connector.dart';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:provider/provider.dart';

import 'package:boda_controller/src/globals.dart';

class ConnectButton extends StatelessWidget {
  final DiscoveredDevice device;

  const ConnectButton({Key? key, required this.device}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Consumer2<BleDeviceConnector, ConnectionStateUpdate>(
      builder: (context, deviceConnector, connectionStateUpdate, child) {
        final isConnected = connectionStateUpdate.connectionState == DeviceConnectionState.connected;

        return ElevatedButton(
          onPressed: () {
            if (isConnected) {
              deviceConnector.disconnect(device.id);
              connectedDeviceId = '';
            } else {
              deviceConnector.connect(device.id);
              connectedDeviceId = device.id;
            }
          },
          style: ElevatedButton.styleFrom(
            backgroundColor: isConnected ? Colors.red : Colors.green,
          ),
          child: Text(isConnected ? 'Disconnect' : 'Connect'),

        );
      },
    );
  }
}