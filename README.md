# Airbag-Simulationsprojekt mit dem Raspberry Pi

Dieses Projekt simuliert ein Airbag-System mithilfe eines Raspberry Pi. Es verwendet einen **MPU6050-Beschleunigungssensor**, einen **Ultraschallsensor (HC-SR04)** und einen **Servomotor**.

## 🚀 Projektziel

Ziel ist es, das Verhalten eines Airbag-Systems bei einem Aufprall nachzubilden – ähnlich wie in echten Fahrzeugen.

## ⚙️ Verwendete Hardware

- Raspberry Pi (beliebiges Modell)
- HC-SR04 Ultraschallsensor
- MPU6050 Beschleunigungssensor
- Servomotor (z. B. SG90)
- Jumper-Kabel, Breadboard

## 📦 Installation

1. Notwendige Python-Bibliotheken installieren:
   ```bash
   pip install mpu6050-raspberrypi
## Funktionsweise
1. Der Ultraschallsensor misst den Abstand zu einem Objekt.
2. Der MPU6050-Sensor misst die Beschleunigung des Systems.
3. Wenn der Abstand sehr gering ist (z. B. < 5 cm) und eine starke Beschleunigung (z. B. > 12 m/s²) erkannt wird, wird dies als Kollision interpretiert.
4. Der Servomotor wird aktiviert, um das Auslösen eines Airbags zu simulieren.

## Kalman Filter 
Die Beschleunigungsdaten werden durch einen Kalman-Filter geglättet, um Messrauschen zu reduzieren und zuverlässigere Entscheidungen zu ermöglichen.

## Projektteam
Eylül Önsal
Kader Yeşilbağ
Nazlı Demir
