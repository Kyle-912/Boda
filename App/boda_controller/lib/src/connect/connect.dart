// ignore_for_file: prefer_const_constructors, prefer_const_literals_to_create_immutables

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:boda_controller/src/connect/ble_status.dart';
import 'package:boda_controller/src/connect/device_list.dart';

class ConnectPage extends StatefulWidget {
  @override
  _ConnectPageState createState() => _ConnectPageState();
}

class _ConnectPageState extends State<ConnectPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      // Use Consumer widget to listen to changes in BleStatus
      body: Consumer<BleStatus?>(
        builder: (_, status, __) {
          if (status == BleStatus.ready) {
            // Return DeviceListScreen when BLE is ready
            return DeviceListPage();
          } else {
            // Return BleStatusScreen for other statuses
            return BleStatusPage(status: status ?? BleStatus.unknown);
          }
        },
      ),
    );
  }
}