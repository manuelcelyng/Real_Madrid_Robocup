/// Desarrollado por Julián Ocampo Vélez
/// Sistemas embebidos - 2023-2
/// Ingeniería electrónica
/// Universidad de Antioquia

import 'package:app_control/routes/routes_app.dart';
import 'package:flutter/material.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Control RoboCup',
      initialRoute: 'control',
      routes: appRoutes,
    );
  }
}
