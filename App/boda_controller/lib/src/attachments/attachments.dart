// ignore_for_file: prefer_const_constructors, prefer_const_literals_to_create_immutables

import 'package:flutter/material.dart';

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
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'UI to show control for each attachment',
              style: TextStyle(fontSize: 20),
            ),
          ],
        ),
      ),
    );
  }
}
