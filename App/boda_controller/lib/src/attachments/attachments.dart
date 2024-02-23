import 'package:flutter/material.dart';
import 'package:boda_controller/src/globals.dart';

class AttachmentWrapper extends StatefulWidget {
  @override
  _AttachmentWrapperState createState() => _AttachmentWrapperState();
}

class _AttachmentWrapperState extends State<AttachmentWrapper> {
  @override
  Widget build(BuildContext context) {
    // Choose the widget based on the current UI type
    Widget child;
    switch (currentUIType) {
      case UIType.typeA:
        child = _buildTypeAUI();
        break;
      case UIType.typeB:
        child = _buildTypeBUI();
        break;
      case UIType.typeC:
        child = _buildTypeCUI();
        break;
      default:
        child = _buildDefaultUI();
    }

    return Scaffold(
      appBar: AppBar(
        title: Text('Attachment Controller'),
      ),
      body: Center(
        child: child,
      ),
    );
  }

  Widget _buildTypeAUI() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Text(
          'UI for Type A',
          style: TextStyle(fontSize: 20),
        ),
        // Add more widgets for Type A UI
      ],
    );
  }

  Widget _buildTypeBUI() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Text(
          'UI for Type B',
          style: TextStyle(fontSize: 20),
        ),
        // Add more widgets for Type B UI
      ],
    );
  }

  Widget _buildTypeCUI() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Text(
          'UI for Type C',
          style: TextStyle(fontSize: 20),
        ),
        // Add more widgets for Type C UI
      ],
    );
  }

  Widget _buildDefaultUI() {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Text(
          'Default UI',
          style: TextStyle(fontSize: 20),
        ),
        // Add more widgets for the default UI
      ],
    );
  }
}
