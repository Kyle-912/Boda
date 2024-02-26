import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:provider/provider.dart';
import 'src/ble/ble_device_connector.dart';
import 'src/ble/ble_scanner.dart';
import 'src/ble/ble_status_monitor.dart';
import 'src/ble/ble_logger.dart';
import 'src/app.dart';
import 'src/settings/settings_controller.dart';
import 'src/settings/settings_service.dart';



void main() async {
  // Set up the SettingsController, which will glue user settings to multiple
  // Flutter Widgets.
  WidgetsFlutterBinding.ensureInitialized();
  final _ble = FlutterReactiveBle();
  final _bleLogger = BleLogger(ble: _ble);
  final _bleScanner = BleScanner(ble: _ble, logMessage: _bleLogger.addToLog);
  final _bleMonitor = BleStatusMonitor(_ble);
  final _bleConnector = BleDeviceConnector(
    ble: _ble,
    logMessage: _bleLogger.addToLog,
  );

  final settingsController = SettingsController(SettingsService());

  // Load the user's preferred theme while the splash screen is displayed.
  // This prevents a sudden theme change when the app is first displayed.
  await settingsController.loadSettings();

  // Run the app
  // Pass in providers for the flutter reactive ble services. The services provided by the reactive ble are acessible accross the app
  // Pass in the SettingsController. The app listens to the
  // SettingsController for changes, then passes it further down to the
  // SettingsView.
  runApp(
    MultiProvider(
      providers: [
        Provider.value(value: _ble),
        Provider.value(value: _bleScanner),
        Provider.value(value: _bleMonitor),
        Provider.value(value: _bleConnector),
        Provider.value(value: _bleLogger),
        StreamProvider<BleScannerState?>(
          create: (_) => _bleScanner.state,
          initialData: const BleScannerState(
            discoveredDevices: [],
            scanIsInProgress: false,
          ),
        ),
        StreamProvider<BleStatus?>(
          create: (_) => _bleMonitor.state,
          initialData: BleStatus.unknown,
        ),
        StreamProvider<ConnectionStateUpdate>(
          create: (_) => _bleConnector.state,
          initialData: const ConnectionStateUpdate(
            deviceId: 'Unknown device',
            connectionState: DeviceConnectionState.disconnected,
            failure: null,
          ),
        ),
      ],
      child: MyApp( settingsController: settingsController)
    ),
  );
}
