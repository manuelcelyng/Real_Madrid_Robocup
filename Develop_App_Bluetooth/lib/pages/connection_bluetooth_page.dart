import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:flutter/material.dart';
import 'package:permission_handler/permission_handler.dart';

// Conexión a bluetooth módulo HC06
class ConnectionBluetoothPage extends StatefulWidget {
  const ConnectionBluetoothPage({Key? key}) : super(key: key);

  @override
  State<ConnectionBluetoothPage> createState() =>
      _ConnectionBluetoothPageState();
}

class _ConnectionBluetoothPageState extends State<ConnectionBluetoothPage> {
  List<BluetoothDevice> devices = []; // Lista de dispositivos encontrados
  late BluetoothConnection connection; // Estado de conexión de los dispositivos
  bool isLoading =
      false; // Indica que se está cargando para mostrar pantalla de carga
  bool connected = false; // Indica que se ha conectado un dispositivo
  BluetoothState bluetoothState =
      BluetoothState.UNKNOWN; // Estado del bluetooth
  final FlutterBluetoothSerial _bluetooth = FlutterBluetoothSerial.instance;

  // Buscar dispositivos
  Future<void> scanDevices() async {
    // Solicitud de permisos necesarios
    Map<Permission, PermissionStatus> permissionStatus = await [
      Permission.bluetoothScan,
      Permission.bluetoothAdvertise
    ].request();

    if ((permissionStatus[Permission.bluetoothScan] ==
            PermissionStatus.granted) &&
        (permissionStatus[Permission.bluetoothAdvertise] ==
            PermissionStatus.granted)) {
      FlutterBluetoothSerial.instance.state.then((state) async {
        if (state == BluetoothState.STATE_ON) {
          // Son necesario estos permisos para lograr  buscar dispositivos de lo contrario la apliación se cerrará y no es posible encontrar dispositivos
          await Permission.location.request();
          await Permission.bluetoothConnect
              .request(); // Permiso de conectividad
          var statusLocation = await Permission.location.status;
          var statusConnect = await Permission.bluetoothConnect.status;
          if (statusLocation.isGranted && statusConnect.isGranted) {
            setState(() {
              isLoading = true;
              devices = [];
            });
            _bluetooth.startDiscovery().listen((event) {
              // Busqueda de los dispositivos y se adiciona a una lista para ser visualizados
              print(
                  'Dispositivo encontrado: ${event.device.name} (${event.device.address})');
              if (!devices
                      .any((device) => device.address == event.device.address) &
                  (event.device.name != null)) {
                setState(() {
                  devices.add(event.device);
                });
              }
            });

            // Despues de 8 segundos se cancela la busqueda para no quedarse mucho tiempo buscando y tener control de la busqueda
            Future.delayed(const Duration(seconds: 3), () {
              FlutterBluetoothSerial.instance.cancelDiscovery();
              setState(() {
                isLoading = false;
              });
            });
          } else {
            if (!mounted) return;
            ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
              content:
                  Text('Verifica los permisos de bluetooth y localización'),
              duration: Duration(seconds: 2),
            ));
          }
        } else {
          ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
            content: Text('Active el Bluetooth'),
            duration: Duration(seconds: 2),
          ));
        }
      });
    } else {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
        content: Text('Active los persmisos de la app'),
        duration: Duration(seconds: 2),
      ));
    }
  }

  // Conectar a algún dispositivo encontrado
  Future<void> _connectToDevice(BluetoothDevice device) async {
    setState(() {
      connected = true;
    });

    // Conexón a dispositvo
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

  void disconnectDevice() {
    try {
      connection.close();
    } catch (e) {
      print("No se ha inicializado el bluetooth");
    }
  }

  @override
  void initState() {
    super.initState();
  }

  // Parte gráfica para conectar y mostrar dispositivos disponibles
  @override
  Widget build(BuildContext context) {
    const Color colorBlue = Color(0xff00145d);
    return Scaffold(
      body: Stack(
        children: [
          Container(
            color: const Color.fromARGB(255, 243, 243, 243),
            child: Column(
              children: [
                Container(
                  height: 100,
                  decoration: const BoxDecoration(
                    color: colorBlue,
                    borderRadius: BorderRadius.vertical(
                      bottom: Radius.circular(30),
                    ),
                  ),
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Padding(
                        padding: const EdgeInsets.only(top: 20),
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            const Text(
                              "Real Madrid",
                              style:
                                  TextStyle(fontSize: 25, color: Colors.white),
                            ),
                            Padding(
                              padding: const EdgeInsets.only(left: 10),
                              child: Image.asset(
                                'assets/escudo.png',
                                width: 40,
                                height: 40,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 20),
                SizedBox(
                  width: 250,
                  height: 50,
                  child: ElevatedButton(
                    onPressed: () => scanDevices(),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.green,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(10),
                      ),
                      elevation: 5,
                    ),
                    child: const Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.bluetooth),
                        Text(
                          'Scan devices',
                          style: TextStyle(fontSize: 18),
                        )
                      ],
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
                SizedBox(
                  width: 200,
                  height: 50,
                  child: ElevatedButton(
                    onPressed: disconnectDevice,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.red,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(10),
                      ),
                      elevation: 5,
                    ),
                    child: const Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(Icons.bluetooth_disabled),
                        Text(
                          'Disconnect',
                          style: TextStyle(fontSize: 18),
                        )
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 20),
              ],
            ),
          ),
          Visibility(
              visible: isLoading, child: _loading("Buscando Dispositivos ...")),
          Visibility(visible: connected, child: _loading("Conectando ...")),
        ],
      ),
    );
  }

  // Pantalla de carga
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
