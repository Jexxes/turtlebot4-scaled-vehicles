# Flashing the SD Card for Raspberry Pi on Turtlebot 4

## Step 1: Gathering the Required Materials
### List of materials needed:
* SD Card (16GB Min. Recommended, make sure it's a fast one)
* Allen wrench (swedish: insexnyckel)
* A small plastic tool for getting the card out of the Raspberry Pi

## Step 2: Downloading the Turtlebot Image
- Download the image from the official website [here](http://download.ros.org/downloads/turtlebot4/)

## Step 3: Remove the SD Card from the Turtlebot 4
- Use the allen wrench to remove the 4 top screws on the Turtlebot. 
- Remove the 4 rods that was held in place by the top screws. 
- Detach the USB chords connected to the Raspberry Pi
- Lift the top lid off carefully
- You can use a plastic tool or your fingers to remove the SD Card 

<img src="../../../Assets/Images/Turtlebot4/Hardware/SD_Card_RPi.png" width="300">

## Step 4: Preparing the SD Card
- Download a SD-card flashing software such as Balena Etcher [here](https://etcher.balena.io/#download-etcher)
- Run Balena Etcher and choose the OS Image to flash onto the SD Card. 

## Step 6 Re-assemble the Turtlebot: 
- Re-insert the SD-card in the Raspberry Pi
- Assemble the Turtlebot
- Place the Turtlebot on its doc and power it back on 

## Step 7: Set up the Turtlebot 
### Install Turtlebot: 
- - Install the Turtlebot using [Discovery Server](../../Setup/Network-Discovery-Server)
- - Install the Turtlebot using [CycloneDDS](../../Setup//Network-Cyclone-DDS)

