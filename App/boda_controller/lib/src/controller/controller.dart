// ignore_for_file: prefer_const_constructors
import 'dart:async';
import 'package:boda_controller/src/ble/ble_device_connector.dart';
import 'package:boda_controller/src/ble/ble_logger.dart';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:provider/provider.dart';
import 'package:boda_controller/src/globals.dart';



class ControllerPage extends StatelessWidget {
  const ControllerPage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) =>
      Consumer4<FlutterReactiveBle,BleDeviceConnector, ConnectionStateUpdate, BleLogger>(
        builder: (_, ble, deviceConnector, connectionStateUpdate, bleLogger, __) => _Controller(
          ble: ble,
          bleConnector: deviceConnector,
          bleUpdate: connectionStateUpdate,
          bleLogger: bleLogger,
        ),
      );
}

class _Controller extends StatefulWidget {
    const _Controller({
      required this.ble,
      required this.bleConnector,
      required this.bleUpdate,
      required this.bleLogger,
    });

    final FlutterReactiveBle ble;
    final BleDeviceConnector bleConnector;
    final ConnectionStateUpdate bleUpdate;
    final BleLogger bleLogger;
  @override
  _ControllerState createState() => _ControllerState();
}

class _ControllerState extends State<_Controller> {
  Uuid serviceUuId = Uuid.parse('ffe0');
  Uuid charUuId = Uuid.parse('ffe1');

  late Timer _statusTimer; 
  String _statusString = '';
  String _bleMsg = '';
  String _bleRes = '';

  bool _shoulderVerticalUp = false;
  bool _shoulderVerticalDown = false;

  bool _shoulderRotationClockwise = false;
  bool _shoulderRotationCounterClockwise = false;
  
  bool _elbowUp = false;
  bool _elbowDown = false;

  double _shoulderVertical = 1.0; // Initial slider value
  double _shoulderRotation = 1.0; // Initial slider value
  double _elbowVertical = 1.0; // Initial slider value

  // Method to toggle shoulder vertical up
  void _toggleShoulderVerticalUp() {
    setState(() {
      _shoulderVerticalUp = !_shoulderVerticalUp;
    });
    // print("SVU: $_shoulderVerticalUp");
  }

  // Method to toggle shoulder vertical down
  void _toggleShoulderVerticalDown() {
    setState(() {
      _shoulderVerticalDown = !_shoulderVerticalDown;
    });
    // print("SVD: $_shoulderVerticalDown");
  }

  // Method to toggle shoulder rotation clockwise
  void _toggleShoulderRotationClockwise() {
    setState(() {
      _shoulderRotationClockwise = !_shoulderRotationClockwise;
    });
  }

  // Method to toggle shoulder rotation counter-clockwise
  void _toggleShoulderRotationCounterClockwise() {
    setState(() {
      _shoulderRotationCounterClockwise = !_shoulderRotationCounterClockwise;
    });
  }

  // Method to toggle elbow up
  void _toggleElbowUp() {
    setState(() {
      _elbowUp =  !_elbowUp;
    });
    // print("ED: ${_elbowUp}");
  }

  // Method to toggle elbow down
  void _toggleElbowDown() {
    setState(() {
      _elbowDown = !_elbowDown;
    });
  }

  void _updateBleMsg(){
    if(widget.bleUpdate.connectionState == DeviceConnectionState.connected) {
      _bleMsg = '';
      _bleMsg += _shoulderVerticalUp == true ? "U" : 
                _shoulderVerticalDown == true ? "D" : 
                "N";
      _bleMsg += String.fromCharCode(_shoulderVertical.toInt());
      _bleMsg += _shoulderRotationClockwise == true ? "F" : 
                _shoulderRotation == true ? "B" : 
                "N";
      _bleMsg += String.fromCharCode(_shoulderRotation.toInt());
      _bleMsg += _elbowUp== true ? "F" : 
                _elbowDown == true ? "B" : 
                "N";
      _bleMsg += String.fromCharCode(_elbowVertical.toInt());
      _bleMsg += String.fromCharCode(0);
      _bleMsg += String.fromCharCode(13);
      _bleMsg += String.fromCharCode(10);
      // print(_bleMsg);
      // print(_bleMsg.codeUnits);
    }
  }

  void _sendBleMsg(){
    if(widget.bleUpdate.connectionState == DeviceConnectionState.connected){
      // send the data
      final characteristic = QualifiedCharacteristic(serviceId: serviceUuId, characteristicId: charUuId, deviceId: connectedDeviceId); 
      widget.ble.writeCharacteristicWithoutResponse(characteristic, value: _bleMsg.codeUnits);
    }
  }

  void _updateStatusString() {
    if(widget.bleUpdate.connectionState == DeviceConnectionState.connected) {
      _statusString = '\nSVU: $_shoulderVerticalUp, \n'
                      'SVD: $_shoulderVerticalDown, \n'
                      'SRC: $_shoulderRotationClockwise, \n'
                      'SRCC: $_shoulderRotationCounterClockwise, \n'
                      'EU: $_elbowUp, \n'
                      'ED: $_elbowDown, \n'
                      'Slider1: ${_shoulderVertical.toStringAsFixed(2)}, \n'
                      'Slider2: ${_shoulderRotation.toStringAsFixed(2)}, \n'
                      'Slider3: ${_elbowVertical.toStringAsFixed(2)} \n';
      _updateBleMsg();
      _sendBleMsg();
      // print(_statusString);
    }
  }

  void readBodaResponse(dynamic data) {
    
    String text = String.fromCharCodes(data);
    print(text[0]);
    if (text[0] == 'A'){
      currentUITypeNotifier.value = UIType.typeA;
    } else if (text[0] == 'B'){
      currentUITypeNotifier.value = UIType.typeB;
    } else if (text[0] == 'C'){
      currentUITypeNotifier.value = UIType.typeC;
    } else {
      currentUITypeNotifier.value = UIType.typeDefault;
    }

  }

  @override
  void initState() {
    super.initState();
    _statusTimer = Timer.periodic(Duration(milliseconds: 1000), (timer) {
      _updateStatusString();
    });
  }

  @override
  void dispose() {
    _statusTimer.cancel(); // Cancel the timer when the widget is disposed
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
  final isConnected = widget.bleUpdate.connectionState == DeviceConnectionState.connected;
  if(isConnected){
    final characteristic = QualifiedCharacteristic(serviceId: serviceUuId, characteristicId: charUuId, deviceId: connectedDeviceId);
    widget.ble.subscribeToCharacteristic(characteristic).listen((data) {
      readBodaResponse(data);
    }, onError: (dynamic error) {
    });
  } 
  
  return Scaffold(
    appBar: AppBar(
      title: Text('Main Controller'),
    ),
    body: SafeArea(
      child: Stack(
        children: [
          IgnorePointer(
            ignoring: !isConnected, // Ignore interactions when not connected
            child:             Padding(
              padding: const EdgeInsets.all(5.0),
              child: Center(
                child: Padding(
                  padding: const EdgeInsets.all(5.0),
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      // Left Column for Sliders
                      Expanded(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.start,
                          children: [
                            Text('Speed',
                              style: TextStyle(fontSize: 14, 
                              fontWeight: FontWeight.bold)),
                            _buildSlider(_shoulderVertical, (newValue) {
                              setState(() => _shoulderVertical = newValue);
                            }, "Shoulder - V"),
                            _buildSlider(_shoulderRotation, (newValue) {
                              setState(() => _shoulderRotation = newValue);
                            }, "Shoulder - R"),
                            _buildSlider(_elbowVertical, (newValue) {
                              setState(() => _elbowVertical = newValue);
                            }, "Elbow"),
                          ],
                        ),
                      ),
                      Expanded(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Padding(
                              padding: const EdgeInsets.symmetric(horizontal: 8.0), // Adjust based on your design needs
                              child: _buildStopButton(),
                            ),
                          ],
                        ),
                      ),
                      // Right Column for Counters and Buttons
                      Expanded(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.start,
                          children: [
                            Text('Movement',
                              style: TextStyle(fontSize: 14, 
                              fontWeight: FontWeight.bold)),
                            _buildVerticalControls(
                              _toggleShoulderVerticalUp, 
                              _toggleShoulderVerticalDown, 
                              ' ',
                              "Shoulder - V"),
                            _buildHorizontalControls(
                              _toggleShoulderRotationCounterClockwise,
                              _toggleShoulderRotationClockwise, 
                             ' ',
                              "Shoulder - R"),
                            _buildVerticalControls(
                              _toggleElbowUp,
                              _toggleElbowDown,
                              ' ',
                              "Elbow"),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
          Visibility(
            visible: !isConnected, // Show the overlay when NOT connected
            child: Container(
              width: MediaQuery.of(context).size.width, 
              height: MediaQuery.of(context).size.height, 
              child: Center(
                child: Container(
                  padding: EdgeInsets.all(20),
                  color: Colors.red, // Red banner background
                  child: Text(
                    'You must connect to an arm before continuing',
                    style: TextStyle(color: Colors.white, fontSize: 16),
                    textAlign: TextAlign.center,
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    ),
  );
}

Widget _buildStopButton() {
  return ElevatedButton(
    onPressed: () {} ,
    style: ElevatedButton.styleFrom(
      shape: CircleBorder(), 
      padding: EdgeInsets.all(20), 
    ),
    child: Icon(Icons.stop), 
  );
}

 Widget _buildSlider(double value, ValueChanged<double> onChanged, String label) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      crossAxisAlignment: CrossAxisAlignment.start, // Align children to the start of the cross axis
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.start, // Align to the left
          children: [
            Text(label, style: TextStyle(fontSize: 16, fontWeight: FontWeight.normal)), // Slider Label
          ],
        ),
        Slider(
          value: value,
          min: 1, // Minimum value set to 0 RPM
          max: 255, // Maximum value set to 450 RPM
          divisions: 255, 
          label: '${(value/ 255 * 450).floor()} RPM', // Display the value in RPM
          onChanged: onChanged,
        )
      ],
    );
  }


  Widget _buildHorizontalControls(VoidCallback toggleCCW, VoidCallback toggleCW, String thirdCounterValue, String label) {
  return Column(
    mainAxisAlignment: MainAxisAlignment.center,
    children: [
      Row(
        mainAxisAlignment: MainAxisAlignment.start, // Align to the left
        children: [
          Text(label, style: TextStyle(fontSize: 16, fontWeight: FontWeight.normal)), // Slider Label
        ],
      ),
      Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Listener(
            onPointerDown: (_) { toggleCCW();},
            onPointerUp: (_) { toggleCCW();},
            child: ElevatedButton(
              onPressed: () {}, 
              child: Icon(Icons.arrow_left),
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8.0),
            child: Text(thirdCounterValue),
          ),
          Listener(
            onPointerDown: (_) { toggleCW();},
            onPointerUp: (_) { toggleCW();},
            child: ElevatedButton(
              onPressed: () {},
              child: Icon(Icons.arrow_right),
            ),
          ),
        ],
      ),
    ],
  );
}

  Widget _buildVerticalControls(VoidCallback toggleUp, VoidCallback toggleDown, String counterValue, String label) {
  return Column(
    mainAxisAlignment: MainAxisAlignment.center,
    children: [
      Row(
        mainAxisAlignment: MainAxisAlignment.start, // Align to the left
        children: [
          Text(label, style: TextStyle(fontSize: 16, fontWeight: FontWeight.normal)), // Slider Label
        ],
      ),
      Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Listener(
            onPointerDown: (_) => toggleUp(),
            onPointerUp: (_) => toggleUp(),
            child: ElevatedButton(
              onPressed: () {}, // You might keep this empty if the button press is fully handled by the Listener
              child: Icon(Icons.arrow_upward),
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8.0),
            child: Text(counterValue),
          ),
          Listener(
            onPointerDown: (_) => toggleDown(),
            onPointerUp: (_) => toggleDown(),
            child: ElevatedButton(
              onPressed: () {}, 
              child: Icon(Icons.arrow_downward),
            ),
          ),
        ],
      )
    ],
  );
}


}
