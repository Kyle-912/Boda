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
    return Scaffold(
      appBar: AppBar(
        title: Text('Controller Page'),
      ),
      body: Center(
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              // Left Column for Sliders
              Expanded(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildSlider(_sliderValue1, (newValue) {
                      setState(() => _sliderValue1 = newValue);
                    }),
                    _buildSlider(_sliderValue2, (newValue) {
                      setState(() => _sliderValue2 = newValue);
                    }),
                    _buildSlider(_sliderValue3, (newValue) {
                      setState(() => _sliderValue3 = newValue);
                    }),
                  ],
                ),
              ),
              // Right Column for Counters and Buttons
              Expanded(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildVerticalControls(_incrementCounter, _decrementCounter, _counter.toString()),
                    _buildHorizontalControls(_decrementThirdCounter, _incrementThirdCounter, _thirdCounter.toString()),
                    _buildVerticalControls(_incrementSecondCounter, _decrementSecondCounter, _secondCounter.toString()),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSlider(double value, ValueChanged<double> onChanged) {
    return Slider(
      value: value,
      min: 0,
      max: 100,
      divisions: 100,
      label: value.round().toString(),
      onChanged: onChanged,
    );
  }

  Widget _buildHorizontalControls(VoidCallback onIncrementThird, VoidCallback onDecrementThird, String thirdCounterValue) {
  return Row(
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
    ],
  );
}


  Widget _buildVerticalControls(VoidCallback onIncrement, VoidCallback onDecrement, String counterValue) {
    return Row(
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
    );
  }
}
