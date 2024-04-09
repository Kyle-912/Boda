import 'package:flutter/material.dart';
import 'package:boda_controller/src/globals.dart'; // Assuming this is where pointsNotifier is defined

class TeachPage extends StatefulWidget {
  @override
  _TeachPageState createState() => _TeachPageState();
}

class _TeachPageState extends State<TeachPage> {

  void _deletePoint(int index) {
    pointsNotifier.value = List.from(pointsNotifier.value)..removeAt(index);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Teach Page'),
      ),
      body: Center(
        child: ValueListenableBuilder<List<String>>(
          valueListenable: pointsNotifier,
          builder: (context, points, child) {
            if (points.isEmpty) {
              return Text(
                'No saved positions available',
                style: TextStyle(fontSize: 20),
              );
            } else {
              return ListView.builder(
                itemCount: points.length,
                itemBuilder: (context, index) {
                  return ListTile(
                    title: Text(
                      points[index],
                      style: TextStyle(fontSize: 20),
                    ),
                    trailing: IconButton(
                      icon: Icon(Icons.delete),
                      onPressed: () {
                        _deletePoint(index);
                      },
                    ),
                  );
                },
              );
            }
          },
        ),
      ),
    );
  }
}
