// Global variable to control the UI type
import 'package:flutter/material.dart';

enum UIType { typeA, typeB, typeC, typeDefault }
ValueNotifier<UIType> currentUITypeNotifier = ValueNotifier<UIType>(UIType.typeDefault);

String connectedDeviceId = '';
ValueNotifier<List<String>> pointsNotifier = ValueNotifier<List<String>>([]);
int pointIdx = 1;
