import 'package:flutter/material.dart';

class MyBottomNavBar extends StatelessWidget {
  final int selectedIndex;
  final Function(int) onItemTapped;

  const MyBottomNavBar({Key? key, required this.selectedIndex, required this.onItemTapped}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return BottomNavigationBar(
      items: const <BottomNavigationBarItem>[
        BottomNavigationBarItem(icon: Icon(Icons.gamepad), label: 'Control'),
        BottomNavigationBarItem(icon: Icon(Icons.add), label: 'Atachment'),
        BottomNavigationBarItem(icon: Icon(Icons.design_services), label: 'Teach'),
        BottomNavigationBarItem(icon: Icon(Icons.menu_book), label: 'Logs'),
      ],
      currentIndex: selectedIndex,
      selectedItemColor: Theme.of(context).colorScheme.secondary,
      unselectedItemColor: Colors.grey,
      showUnselectedLabels: true, // Ensure labels are shown for unselected items
      showSelectedLabels: true,
      onTap: onItemTapped,
    );
  }
}

