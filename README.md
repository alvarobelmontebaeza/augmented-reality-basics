# AUGMENTED-REALITY-BASICS

This package provides a basic AR functionality to be used with the duckietown platform

## Instructions

### 1) Clone the repository

git clone https://github.com/alvarobelmontebaeza/augmented-reality-basics

### 2) Move into the root diretory of the repository

### 3) Select which map file to load

cd packages/augmented_reality_basics/launch/

Open the launchfile in your favourite text editor, and change the map_file argument to load the desired map

### 4) Build the package in your Duckiebot

dts devel build -f -H <HOSTNAME>.local

### 5) Run the node

dts devel run -H <HOSTNAME>.local

### 6) In another terminal, open rqt_image_view and select the correct topic to visualize the AR image

dts start_gui_tools <HOSTNAME>

rqt_image_view

### 7) Enjoy!

NOTE: For the calibration_pattern and lane maps, you will need to locate the duckiebot accurately to see
good results!
