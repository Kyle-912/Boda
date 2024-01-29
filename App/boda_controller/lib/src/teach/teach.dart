// ignore_for_file: prefer_const_constructors, prefer_const_literals_to_create_immutables

import 'package:flutter/material.dart';

class TeachPage extends StatefulWidget {
  @override
  _TeachPageState createState() => _TeachPageState();
}

class _TeachPageState extends State<TeachPage> {
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
        title: Text('Teach Page'),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'UI to teach robot different set points',
              style: TextStyle(fontSize: 20),
            ),
          ],
        ),
      ),
    );
  }
}
