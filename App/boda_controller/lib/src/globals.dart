// Global variable to control the UI type
import 'package:flutter/material.dart';

enum UIType { typeA, typeB, typeC, typeDefault }
ValueNotifier<UIType> currentUITypeNotifier = ValueNotifier<UIType>(UIType.typeDefault);

String connectedDeviceId = '';

