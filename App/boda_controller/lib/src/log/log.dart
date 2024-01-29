// ignore_for_file: prefer_const_constructors, prefer_const_literals_to_create_immutables

import 'package:flutter/material.dart';

class LogPage extends StatefulWidget {
  @override
  _LogPageState createState() => _LogPageState();
}

class _LogPageState extends State<LogPage> {
  int _counter = 0; // Initial counter value

  void _incrementCounter() {
    setState(() {
      _counter++; // Increment counter
    });
  }

  void _decrementCounter() {
    setState(() {
      if (_counter > 0) _counter--; // Decrement counter but never go below 0
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Logs Page'),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'UI to show logs of robot interactions and errors',
              style: TextStyle(fontSize: 20),
            ),
          ],
        ),
      ),
    );
  }
}
