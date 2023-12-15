import 'package:app_control/pages/connection_bluetooth_page.dart';
import 'package:app_control/pages/control_page.dart';
import 'package:flutter/material.dart';

final Map<String, Widget Function(BuildContext)> appRoutes = {
  'connection': (_) => const ConnectionBluetoothPage(),
  'control': (_) => const ControlPage(),
};
