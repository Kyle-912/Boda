import 'dart:async';
import 'package:boda_controller/src/connect/connect_button.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import 'package:boda_controller/src/ble/ble_logger.dart';
import 'package:boda_controller/src/ble/ble_scanner.dart';

import 'package:boda_controller/src/widgets.dart';





class DeviceListPage extends StatelessWidget {
  const DeviceListPage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) =>
      Consumer3<BleScanner, BleScannerState?, BleLogger>(
        builder: (_, bleScanner, bleScannerState, bleLogger, __) => _DeviceList(
          scannerState: bleScannerState ??
              const BleScannerState(
                discoveredDevices: [],
                scanIsInProgress: false,
              ),
          startScan: bleScanner.startScan,
          stopScan: bleScanner.stopScan,
          toggleVerboseLogging: bleLogger.toggleVerboseLogging,
          verboseLogging: bleLogger.verboseLogging,
        ),
      );
}

class _DeviceList extends StatefulWidget {
  const _DeviceList({
    required this.scannerState,
    required this.startScan,
    required this.stopScan,
    required this.toggleVerboseLogging,
    required this.verboseLogging,
  });

  final BleScannerState scannerState;
  final void Function(List<Uuid>) startScan;
  final VoidCallback stopScan;
  final VoidCallback toggleVerboseLogging;
  final bool verboseLogging;

  @override
  _DeviceListState createState() => _DeviceListState();
}

class _DeviceListState extends State<_DeviceList> {
  final String _uuidBoda = 'ffe0';
  Timer? _scanTimer;

  @override
  void initState() {
    super.initState();
  }

  @override
  void dispose() {
    widget.stopScan();
    _scanTimer?.cancel();
    super.dispose();
  }

  void _startScanning() {
    if (widget.scannerState.scanIsInProgress) {
      widget.stopScan(); // Stop any ongoing scan
      _scanTimer?.cancel(); // Cancel any existing timer
    }
    widget.startScan([Uuid.parse(_uuidBoda)]);
    _scanTimer = Timer(const Duration(seconds: 20), widget.stopScan);
  }

  @override
  Widget build(BuildContext context) => Scaffold(
        appBar: AppBar(
        title: const Text('Connect to Device'),
        actions: <Widget>[
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _startScanning,
          ),
        ],
      ),
        body: Column(
          children: [
            Flexible(
              child: ListView(
                children: [
                  ListTile(
                    title: Text(
                      !widget.scannerState.scanIsInProgress
                          ? 'Tap the refresh button to scan for devices'
                          : 'Scanning for devices',
                    ),
                    trailing: (widget.scannerState.scanIsInProgress ||
                            widget.scannerState.discoveredDevices.isNotEmpty)
                        ? Text(
                            'count: ${widget.scannerState.discoveredDevices.length}',
                          )
                        : null,
                  ),
                  ...widget.scannerState.discoveredDevices
                      .map(
                        (device) => ListTile(
                          title: Text(
                            device.name.isNotEmpty ? device.name : "Unnamed",
                          ),
                          subtitle: Text(
                            """
${device.id}
RSSI: ${device.rssi}
${device.connectable.index == 1 ? "Not Connectable" : 
  device.connectable.index == 2 ? "Connectable" : 
  "Unknown Status"}
                            """,
                          ),
                          leading: const BluetoothIcon(),
                          // onTap: () async {
                          //   widget.stopScan();
                          //   await Navigator.push<void>(
                          //     context,
                          //     MaterialPageRoute(
                          //       builder: (_) =>
                          //           DeviceDetailPage(device: device),
                          //     ),
                          //   );
                          // },
                          trailing: ConnectButton(device: device),
                        ),
                      )
                      .toList(),
                ],
              ),
            ),
          ],
        ),
      );
}

