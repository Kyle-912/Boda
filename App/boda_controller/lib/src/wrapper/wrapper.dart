
import 'package:boda_controller/src/attachments/attachments.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:boda_controller/src/nav/nav.dart';
import 'package:boda_controller/src/controller/controller.dart';
import 'package:boda_controller/src/teach/teach.dart';
import 'package:boda_controller/src/log/log.dart';
import 'package:boda_controller/src/connect/connect.dart';


class AppWrapper extends StatefulWidget {
  const AppWrapper({super.key});
  static const String routeName = '/';

  @override
  _AppWrapperState createState() => _AppWrapperState();
}

class _AppWrapperState extends State<AppWrapper> {
  @override
  void initState() {
    super.initState();
  }
  int _selectedIndex = 0;

  final List<Widget> _pages = [
    ControllerPage(),
    AttachmentWrapper(),
    TeachPage(),
    LogPage(),
    ConnectPage(),
  ];

  void _onItemTapped(int index) {
    setState(() {
      _selectedIndex = index;
    });
  }

  @override
  Widget build(BuildContext context) {
    return  PopScope(
    canPop: false,
      child: Scaffold(
        body: IndexedStack(
          index: _selectedIndex,
          children: _pages,
        ),
        bottomNavigationBar: MyBottomNavBar(
          selectedIndex: _selectedIndex,
          onItemTapped: _onItemTapped,
        ),
      ),
    );
  }

  @override
  void dispose() {
    super.dispose();
  }
}
