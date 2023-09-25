import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

class ConnectionBluetoothPage extends StatefulWidget {
  const ConnectionBluetoothPage({Key? key}) : super(key: key);

  @override
  State<ConnectionBluetoothPage> createState() =>
      _ConnectionBluetoothPageState();
}

class _ConnectionBluetoothPageState extends State<ConnectionBluetoothPage> {
  List<BluetoothDevice> devices = [];
  late BluetoothConnection connection;
  bool isLoading = false;
  bool connected = false;

  void scanDevices() {
    setState(() {
      isLoading = true;
      devices = [];
    });

    FlutterBluetoothSerial.instance.startDiscovery().listen(
      (h) {
        print('Dispositivo encontrado: ${h.device.name} (${h.device.address})');
        if (!devices.any((device) => device.address == h.device.address) &
            (h.device.name != null)) {
          setState(() {
            devices.add(h.device);
          });
        }
      },
    );

    Future.delayed(const Duration(seconds: 8), () {
      FlutterBluetoothSerial.instance.cancelDiscovery();
      setState(() {
        isLoading = false;
      });
    });
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    setState(() {
      connected = true;
    });

    try {
      connection = await BluetoothConnection.toAddress(device.address);
      if (!mounted) return;
      Navigator.pushNamed(context, 'control', arguments: connection);
      setState(() {
        connected = false;
      });
      print('Conectado a: ${device.name} (${device.address})');
    } catch (e) {
      print('Error al conectar al dispositivo: $e');
      setState(() {
        connected = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Stack(
          children: [
            Column(
              children: [
                const SizedBox(height: 30),
                GestureDetector(
                  onTap: scanDevices,
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 30),
                    child: Container(
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(10.0),
                        color: const Color.fromARGB(132, 0, 114, 28),
                      ),
                      height: 60,
                      child: const Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Icon(Icons.bluetooth),
                          Text(
                            'Buscar Disposivios ...',
                            style: TextStyle(fontSize: 18),
                          )
                        ],
                      ),
                    ),
                  ),
                ),
                const SizedBox(height: 30),
                Expanded(
                  child: ListView.builder(
                    itemCount: devices.length,
                    itemBuilder: (BuildContext context, int index) {
                      final device = devices[index];
                      return ListTile(
                        title: Text(device.name ?? 'Desconocido'),
                        subtitle: Text(device.address),
                        trailing: const Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(Icons.double_arrow),
                          ],
                        ),
                        onTap: () {
                          _connectToDevice(device);
                        },
                      );
                    },
                  ),
                ),
                GestureDetector(
                  onTap: () {
                    connection.close();
                    print('Conexión cerrada.');
                  },
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 30),
                    child: Container(
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(10.0),
                        color: const Color.fromARGB(132, 163, 0, 0),
                      ),
                      height: 60,
                      width: 250,
                      child: const Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Icon(Icons.bluetooth),
                          Text(
                            'Desconectar',
                            style: TextStyle(fontSize: 18),
                          )
                        ],
                      ),
                    ),
                  ),
                ),
                const SizedBox(height: 20),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Image.asset(
                      'assets/escudo.png',
                      width: 40,
                      height: 40,
                    ),
                    const Padding(
                      padding: EdgeInsets.only(left: 10),
                      child: Text(
                        "Real Madrid",
                        style: TextStyle(fontSize: 30),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 30)
              ],
            ),
            Visibility(
                visible: isLoading,
                child: _loading("Buscando Dispositivos ...")),
            Visibility(visible: connected, child: _loading("Conectando ...")),
          ],
        ),
      ),
    );
  }

  // Future<void> sendWord(String word) async {
  //   try {
  //     final FlutterBluetoothSerial bluetooth = FlutterBluetoothSerial.instance;
  //     final connectedDevices = await bluetooth.getBondedDevices();
  //     if (connectedDevices.isNotEmpty) {
  //       final device = connectedDevices.first;
  //       Uint8List data = Uint8List.fromList(utf8.encode('HOLA GRUPO\n'));
  //       connection.output.add(data);
  //       await connection.output.allSent.then((_) {
  //         print('Mensaje enviado');
  //       });
  //       // await bluetooth.disconnect();
  //     } else {
  //       print('No se encontraron dispositivos Bluetooth emparejados.');
  //     }
  //   } catch (e) {
  //     print('Error al enviar datos por Bluetooth: $e');
  //   }
  // }

  Widget _loading(String textConnect) {
    return Container(
      color: const Color.fromARGB(158, 168, 167, 167),
      width: double.infinity,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          CircularProgressIndicator(color: Theme.of(context).focusColor),
          const SizedBox(height: 30),
          Text(textConnect),
        ],
      ),
    );
  }
}
