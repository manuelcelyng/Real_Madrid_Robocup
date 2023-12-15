import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:flutter/material.dart';

class ControlPage extends StatefulWidget {
  const ControlPage({Key? key}) : super(key: key);

  @override
  State<ControlPage> createState() => _ControlPageState();
}

class _ControlPageState extends State<ControlPage> {
  TextEditingController ctrTurn = TextEditingController();
  TextEditingController ctrDistance = TextEditingController();
  TextEditingController ctrRadius = TextEditingController();
  TextEditingController ctrAngle = TextEditingController();
  TextEditingController ctrAngleDistance = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final BluetoothConnection connection =
        ModalRoute.of(context)!.settings.arguments as BluetoothConnection;
    const Color colorBlue = Color(0xff00145d);

    return Scaffold(
      body: Container(
        height: MediaQuery.of(context).size.height,
        color: const Color.fromARGB(255, 243, 243, 243),
        child: SingleChildScrollView(
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
                            style: TextStyle(fontSize: 25, color: Colors.white),
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
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 20),
                child: Column(
                  children: [
                    Row(
                      children: [
                        Expanded(
                          child: SizedBox(
                            width: 200,
                            child: ElevatedButton(
                              onPressed: () async {
                                Uint8List data =
                                    Uint8List.fromList(utf8.encode('S;0;0\n'));
                                connection.output.add(data);
                                await connection.output.allSent.then((_) {
                                  print('Mensaje enviado');
                                  ScaffoldMessenger.of(context)
                                      .showSnackBar(const SnackBar(
                                    content: Text('¡Command send!'),
                                    duration: Duration(seconds: 2),
                                  ));
                                });
                              },
                              style: ElevatedButton.styleFrom(
                                backgroundColor: Colors.red,
                                shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(10),
                                ),
                                elevation: 5,
                              ),
                              child: const Text('STOP'),
                            ),
                          ),
                        ),
                        const SizedBox(width: 30),
                        Expanded(
                          child: SizedBox(
                            width: 200,
                            child: ElevatedButton(
                              onPressed: () =>
                                  alertDialogDribbling(context, connection),
                              style: ElevatedButton.styleFrom(
                                backgroundColor: Colors.orange,
                                shape: RoundedRectangleBorder(
                                  borderRadius: BorderRadius.circular(10),
                                ),
                                elevation: 5,
                              ),
                              child: const Text('DRIBBLEO'),
                            ),
                          ),
                        ),
                      ],
                    ),

                    cardCommand(ctrTurn, "Turn", "Enter the angle", () async {
                      Uint8List data = Uint8List.fromList(
                          utf8.encode('T;${ctrTurn.text};0\n'));
                      connection.output.add(data);
                      await connection.output.allSent.then((_) {
                        print('Mensaje enviado');
                        ScaffoldMessenger.of(context)
                            .showSnackBar(const SnackBar(
                          content: Text('¡Command send!'),
                          duration: Duration(seconds: 2),
                        ));
                      });
                    }),
                    cardCommand(
                        ctrDistance, "Displacement", "Enter the distance",
                        () async {
                      Uint8List data = Uint8List.fromList(
                          utf8.encode('D;${ctrDistance.text};0\n'));
                      connection.output.add(data);
                      await connection.output.allSent.then((_) {
                        print('Mensaje enviado');
                        ScaffoldMessenger.of(context)
                            .showSnackBar(const SnackBar(
                          content: Text('¡Command send!'),
                          duration: Duration(seconds: 2),
                        ));
                      });
                    }),
                    // 'D;${ctrRadius.text};${ctrAngle.text}\n'
                    cardCommand(
                        ctrRadius, "Circular displacement", "Enter the radius",
                        () async {
                      Uint8List data = Uint8List.fromList(utf8
                          .encode('C;${ctrRadius.text};${ctrAngle.text}\n'));
                      connection.output.add(data);
                      await connection.output.allSent.then((_) {
                        print('Mensaje enviado');
                        ScaffoldMessenger.of(context)
                            .showSnackBar(const SnackBar(
                          content: Text('¡Command send!'),
                          duration: Duration(seconds: 2),
                        ));
                      });
                    }, "Enter the angle", ctrAngle),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<dynamic> alertDialogDribbling(
      BuildContext context, BluetoothConnection connection) {
    return showDialog(
        context: context,
        builder: (BuildContext context) {
          return AlertDialog(
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 5),
                  child: SizedBox(
                      width: 150,
                      child: ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.green,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(10),
                            ),
                            elevation: 5,
                          ),
                          onPressed: () async {
                            Uint8List data =
                                Uint8List.fromList(utf8.encode('A;0;0\n'));
                            connection.output.add(data);
                            await connection.output.allSent.then((_) {
                              print('Mensaje enviado');
                              ScaffoldMessenger.of(context)
                                  .showSnackBar(const SnackBar(
                                content: Text('¡Command send!'),
                                duration: Duration(seconds: 2),
                              ));
                            });
                          },
                          child: const Text("DRIBBLING"))),
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 5),
                  child: SizedBox(
                      width: 150,
                      child: ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.amber,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(10),
                            ),
                            elevation: 5,
                          ),
                          onPressed: () async {
                            Uint8List data =
                                Uint8List.fromList(utf8.encode('K;0;0\n'));
                            connection.output.add(data);
                            await connection.output.allSent.then((_) {
                              print('Mensaje enviado');
                              ScaffoldMessenger.of(context)
                                  .showSnackBar(const SnackBar(
                                content: Text('¡Command send!'),
                                duration: Duration(seconds: 2),
                              ));
                            });
                          },
                          child: const Text("KICK"))),
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 5),
                  child: SizedBox(
                      width: 150,
                      child: ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.red,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(10),
                            ),
                            elevation: 5,
                          ),
                          onPressed: () async {
                            Uint8List data =
                                Uint8List.fromList(utf8.encode('G;0;0\n'));
                            connection.output.add(data);
                            await connection.output.allSent.then((_) {
                              print('Mensaje enviado');
                              ScaffoldMessenger.of(context)
                                  .showSnackBar(const SnackBar(
                                content: Text('¡Command send!'),
                                duration: Duration(seconds: 2),
                              ));
                            });
                          },
                          child: const Text("STOP"))),
                )
              ],
            ),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(10.0),
            ),
          );
        });
  }

  Widget cardCommand(TextEditingController controller, String title,
      String helperText, VoidCallback onPressed,
      [String? helperText2, TextEditingController? controller2]) {
    const Color colorBlue = Color(0xff00145d);
    return Card(
      margin: const EdgeInsets.symmetric(vertical: 15),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
      elevation: 6,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 35, vertical: 25),
        child: Column(
          children: [
            Column(
              children: [
                Text(title, style: const TextStyle(fontSize: 25)),
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 15),
                  child: SizedBox(
                    height: 40,
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Expanded(
                          child: TextFormField(
                            keyboardType: TextInputType.number,
                            cursorColor: colorBlue,
                            decoration: InputDecoration(
                              hintText: helperText,
                              enabledBorder: const UnderlineInputBorder(
                                borderSide: BorderSide(
                                    width: 2, color: Color(0xff464545)),
                              ),
                              focusedBorder: const UnderlineInputBorder(
                                borderSide:
                                    BorderSide(width: 3, color: colorBlue),
                              ),
                            ),
                            controller: controller,
                          ),
                        ),
                        helperText2 == null
                            ? const SizedBox(width: 0)
                            : const SizedBox(width: 30),
                        controller2 == null
                            ? const SizedBox(width: 0)
                            : Expanded(
                                child: TextFormField(
                                  keyboardType: TextInputType.number,
                                  cursorColor: colorBlue,
                                  decoration: InputDecoration(
                                    hintText: helperText2,
                                    enabledBorder: const UnderlineInputBorder(
                                      borderSide: BorderSide(
                                          width: 2, color: Color(0xff464545)),
                                    ),
                                    focusedBorder: const UnderlineInputBorder(
                                      borderSide: BorderSide(
                                          width: 3, color: colorBlue),
                                    ),
                                  ),
                                  controller: controller2,
                                ),
                              ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
            Container(
                width: 150,
                padding: const EdgeInsets.only(top: 20),
                child: ElevatedButton(
                  onPressed: onPressed,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xff00145d),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(10),
                    ),
                    elevation: 5,
                  ),
                  child: const Text("Send"),
                )),
          ],
        ),
      ),
    );
  }
}

  // Widget _buttonsControl(BluetoothConnection connection) {
  // Widget _buttonsControl() {
  //   return Column(
  //     children: [
  //       Row(
  //         mainAxisAlignment: MainAxisAlignment.center,
  //         children: [
  //           GestureDetector(
  //             child: const Icon(
  //               Icons.arrow_upward,
  //               size: 100,
  //             ),
  //             onTap: () {},
  //           ),
  //         ],
  //       ),
  //       Row(
  //         mainAxisAlignment: MainAxisAlignment.spaceBetween,
  //         crossAxisAlignment: CrossAxisAlignment.center,
  //         children: [
  //           const SizedBox(width: 1),
  //           GestureDetector(
  //             child: const Icon(
  //               Icons.arrow_back,
  //               size: 100,
  //             ),
  //             onTap: () {},
  //           ),
  //           const Icon(
  //             Icons.circle,
  //             size: 50,
  //           ),
  //           GestureDetector(
  //             child: const Icon(
  //               Icons.arrow_forward,
  //               size: 100,
  //             ),
  //             onTap: () {},
  //           ),
  //           const SizedBox(width: 1),
  //         ],
  //       ),
  //       Row(
  //         mainAxisAlignment: MainAxisAlignment.center,
  //         children: [
  //           GestureDetector(
  //             child: const Icon(
  //               Icons.arrow_downward,
  //               size: 100,
  //             ),
  //             onTap: () {},
  //           ),
  //         ],
  //       ),
  //     ],
  //   );
  // }

  // Widget _buttonsCommands(BluetoothConnection connection) {
  // Widget _buttonsCommands() {
  //   return Column(
  //     children: [
  //       Row(
  //         mainAxisAlignment: MainAxisAlignment.spaceEvenly,
  //         children: [
  //           const SizedBox(width: 1),
  //           ButtonControl(
  //             colorButton: const [Color(0xFFD4D800), Color(0xFFF2F700)],
  //             textButton: 'Zig Zag',
  //             onTap: () async {
  //               // Uint8List data =
  //               //     Uint8List.fromList(utf8.encode('HOLA GRUPO\n'));
  //               // connection.output.add(data);
  //               // await connection.output.allSent.then((_) {
  //               //   print('Mensaje enviado');
  //               // });
  //             },
  //           ),
  //           ButtonControl(
  //             colorButton: const [Color(0xFFFF3D3D), Color(0xFFBA0000)],
  //             textButton: 'STOP',
  //             onTap: () {},
  //           ),
  //           const SizedBox(width: 1)
  //         ],
  //       ),
  //       const SizedBox(height: 30),
  //       Row(
  //         mainAxisAlignment: MainAxisAlignment.spaceEvenly,
  //         children: [
  //           const SizedBox(width: 1),
  //           ButtonControl(
  //             colorButton: const [Color(0xFF007165), Color(0xFF00AB99)],
  //             textButton: '180°',
  //             onTap: () {},
  //           ),
  //           ButtonControl(
  //             colorButton: const [Color(0xFF120098), Color(0xFF4B32FF)],
  //             textButton: '90°',
  //             onTap: () {},
  //           ),
  //           const SizedBox(width: 1)
  //         ],
  //       ),
  //     ],
  //   );
  // }
