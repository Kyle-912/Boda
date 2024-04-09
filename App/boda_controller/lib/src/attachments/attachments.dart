import 'package:flutter/material.dart';
import 'package:boda_controller/src/globals.dart';

class AttachmentWrapper extends StatefulWidget {
  @override
  _AttachmentWrapperState createState() => _AttachmentWrapperState();
}

class _AttachmentWrapperState extends State<AttachmentWrapper> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Attachment Controller'),
      ),
      body: Center(
        child: ValueListenableBuilder<UIType>(
          valueListenable: currentUITypeNotifier,
          builder: (context, value, child) {
            switch (value) {
              case UIType.typeA:
                return _buildTypeAUI();
              case UIType.typeB:
                return _buildTypeBUI();
              case UIType.typeC:
                return _buildTypeCUI();
              default:
                return _buildDefaultUI();
            }
          },
        ),
      ),
    );
  }

}

  Widget _buildTypeAUI() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Row(
              children: [
                _buildButton()
              ],
            ),
            Row(
              children: [
                _buildButton(),
                _buildButton(),
              ],
            ),
            Row(
              children: [
                _buildButton(),
              ],
            ),
          ],
        ),
            // Second column for diamond pattern
        Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Row(
              children: [
                _buildRoundButton('Y')
              ],
            ),
            Row(
              children: [
                _buildRoundButton('X'),
                _buildRoundButton('B'),
              ],
            ),
            Row(
              children: [
                _buildRoundButton('A'),
              ],
            ),
          ],
        ),
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
          'Please connect an attachment',
          style: TextStyle(fontSize: 20),
        ),
        // Add more widgets for Type B UI
      ],
    );
  }

  Widget _buildButton() {
  return Padding(
    padding: const EdgeInsets.all(8.0),
    child: ElevatedButton(
      onPressed: () {},
      child: Text('Action N'),
    ),
  );
}

  Widget _buildRoundButton(String label) {
  return Padding(
    padding: const EdgeInsets.all(1.0),
    child: ClipOval(
      child: ElevatedButton(
        onPressed: () {},
        child: Container(
          width: 45, // Set the width and height to the same value for a circle
          height: 45,
          alignment: Alignment.center,
          child: Text(label),
        ),
        style: ElevatedButton.styleFrom(
          shape: CircleBorder(), 
        ),
      ),
    ),
  );
}