import 'dart:convert';
import 'dart:typed_data';

import 'package:app_control/widgets/button_control.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:flutter/material.dart';

class ControlPage extends StatelessWidget {
  const ControlPage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final BluetoothConnection connection =
        ModalRoute.of(context)!.settings.arguments as BluetoothConnection;

    return Scaffold(
      body: SafeArea(
          child: Container(
        width: double.infinity,
        height: MediaQuery.of(context).size.height,
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            colors: [Color(0xffC0BEBE), Colors.white],
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
        ),
        child: Row(
          children: [
            Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Padding(
                  padding: EdgeInsets.only(left: 10),
                  child: RotatedBox(
                      quarterTurns: 3,
                      child: Text(
                        "Real Madrid",
                        style: TextStyle(fontSize: 30),
                      )),
                ),
                Padding(
                  padding: const EdgeInsets.only(left: 10, top: 10),
                  child: RotatedBox(
                      quarterTurns: 3,
                      child: Image.asset(
                        'assets/escudo.png',
                        width: 50,
                        height: 50,
                      )),
                ),
              ],
            ),
            Expanded(
              child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buttonsCommands(connection),
                    const SizedBox(height: 120),
                    _buttonsControl(connection),
                  ]),
            ),
          ],
        ),
      )),
    );
  }

  Widget _buttonsControl(BluetoothConnection connection) {
    return Column(
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            GestureDetector(
              child: const Icon(
                Icons.arrow_upward,
                size: 100,
              ),
              onTap: () {},
            ),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            const SizedBox(width: 1),
            GestureDetector(
              child: const Icon(
                Icons.arrow_back,
                size: 100,
              ),
              onTap: () {},
            ),
            const Icon(
              Icons.circle,
              size: 50,
            ),
            GestureDetector(
              child: const Icon(
                Icons.arrow_forward,
                size: 100,
              ),
              onTap: () {},
            ),
            const SizedBox(width: 1),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            GestureDetector(
              child: const Icon(
                Icons.arrow_downward,
                size: 100,
              ),
              onTap: () {},
            ),
          ],
        ),
      ],
    );
  }

  Widget _buttonsCommands(BluetoothConnection connection) {
    return Column(
      children: [
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            const SizedBox(width: 1),
            ButtonControl(
              colorButton: const [Color(0xFFD4D800), Color(0xFFF2F700)],
              textButton: 'Zig Zag',
              onTap: () async {
                Uint8List data =
                    Uint8List.fromList(utf8.encode('HOLA GRUPO\n'));
                connection.output.add(data);
                await connection.output.allSent.then((_) {
                  print('Mensaje enviado');
                });
              },
            ),
            ButtonControl(
              colorButton: const [Color(0xFFFF3D3D), Color(0xFFBA0000)],
              textButton: 'STOP',
              onTap: () {},
            ),
            const SizedBox(width: 1)
          ],
        ),
        const SizedBox(height: 30),
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            const SizedBox(width: 1),
            ButtonControl(
              colorButton: const [Color(0xFF007165), Color(0xFF00AB99)],
              textButton: '180°',
              onTap: () {},
            ),
            ButtonControl(
              colorButton: const [Color(0xFF120098), Color(0xFF4B32FF)],
              textButton: '90°',
              onTap: () {},
            ),
            const SizedBox(width: 1)
          ],
        ),
      ],
    );
  }
}
