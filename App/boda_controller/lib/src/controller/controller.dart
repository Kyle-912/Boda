// ignore_for_file: prefer_const_constructors

import 'package:flutter/material.dart';

class ControllerPage extends StatefulWidget {
  @override
  _ControllerPageState createState() => _ControllerPageState();
}

class _ControllerPageState extends State<ControllerPage> {
  int _counter = 0; // Initial counter value
  int _secondCounter = 0; // Second counter value
  int _thirdCounter = 0; // Third counter value
  double _sliderValue1 = 0; // Initial slider value
  double _sliderValue2 = 0; // Initial slider value
  double _sliderValue3 = 0; // Initial slider value

  void _incrementCounter() {
    setState(() {
      _counter++; // Increment first counter
    });
  }

  void _decrementCounter() {
    setState(() {
      if (_counter > 0) _counter--; // Decrement first counter 
    });
  }

  void _incrementSecondCounter() {
    setState(() {
      _secondCounter++; // Increment second counter
    });
  }

  void _decrementSecondCounter() {
    setState(() {
      if (_secondCounter > 0) _secondCounter--; // Decrement second counter 
    });
  }

  void _incrementThirdCounter() {
    setState(() {
      _thirdCounter++; // Increment first counter
    });
  }

  void _decrementThirdCounter() {
    setState(() {
      _thirdCounter--; // Decrement first counter
    });
  }

  @override
  Widget build(BuildContext context) {
    final Size screenSize = MediaQuery.of(context).size;
    final EdgeInsets safePadding = MediaQuery.of(context).padding;
    return Scaffold(
      appBar: AppBar(
        title: Text('Main Controller'),
      ),
      body: SafeArea(
        child: Padding(
          // Apply padding to respect system UI elements
          padding: EdgeInsets.only(top: safePadding.top, bottom: safePadding.bottom),
          child: Center(
            child: Padding(
              padding: const EdgeInsets.all(10.0),
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
                        _buildSlider(_sliderValue1, (newValue) {
                          setState(() => _sliderValue1 = newValue);
                        }, "Shoulder - V"),
                        _buildSlider(_sliderValue2, (newValue) {
                          setState(() => _sliderValue2 = newValue);
                        }, "Shoulder - R"),
                        _buildSlider(_sliderValue3, (newValue) {
                          setState(() => _sliderValue3 = newValue);
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
                          _incrementCounter, 
                          _decrementCounter, 
                          _counter.toString(),
                          "Shoulder - V"),
                        _buildHorizontalControls(
                          _decrementThirdCounter,
                          _incrementThirdCounter, 
                          _thirdCounter.toString(),
                          "Shoulder - R"),
                        _buildVerticalControls(
                          _incrementSecondCounter,
                          _decrementSecondCounter,
                          _secondCounter.toString(),
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
          min: 0, // Minimum value set to 0 RPM
          max: 450, // Maximum value set to 450 RPM
          divisions: 450, 
          label: '${value.round()} RPM', // Display the value in RPM
          onChanged: onChanged,
        )
      ],
    );
  }


  Widget _buildHorizontalControls(VoidCallback onIncrementThird, VoidCallback onDecrementThird, String thirdCounterValue, String label) {
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
          ElevatedButton(
            onPressed: onIncrementThird, 
            child: Icon(Icons.arrow_left),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8.0),
            child: Text(thirdCounterValue),
          ),
          ElevatedButton(
            onPressed: onDecrementThird, 
            child: Icon(Icons.arrow_right),
          ),
        ]
      ),
    ],
  );
}


  Widget _buildVerticalControls(VoidCallback onIncrement, VoidCallback onDecrement, String counterValue, String label) {
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
            ElevatedButton(
              onPressed: onIncrement,
              child: Icon(Icons.arrow_upward),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 8.0),
              child: Text(counterValue),
            ),
            ElevatedButton(
              onPressed: onDecrement,
              child: Icon(Icons.arrow_downward),
            ),

          ],
        )
      ],
    );
  }
}
