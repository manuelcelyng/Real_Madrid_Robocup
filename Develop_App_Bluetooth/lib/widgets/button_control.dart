import 'package:flutter/material.dart';

class ButtonControl extends StatelessWidget {
  const ButtonControl({
    Key? key,
    required this.colorButton,
    required this.onTap,
    required this.textButton,
  }) : super(key: key);

  final List<Color> colorButton;
  final String textButton;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 90,
        height: 90,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          gradient: LinearGradient(
            colors: colorButton,
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
          boxShadow: [
            BoxShadow(
              color: Colors.grey.withOpacity(0.8),
              offset: const Offset(0, 3),
              blurRadius: 20,
              spreadRadius: 5,
            ),
          ],
        ),
        child: Center(
            child: RotatedBox(
                quarterTurns: 3,
                child: Text(
                  textButton,
                  style: const TextStyle(
                    fontSize: 20,
                    fontWeight: FontWeight.bold,
                  ),
                ))),
      ),
    );
  }
}
