#!/bin/bash
sleep 1s;
gnome-terminal --tab --title="PlotJuggler" --command  "bash -c \"source ~/.bashrc; rosrun plotjuggler plotjuggler --layout ~/catkin_ws_assistive/plotjuggler_layouts/plotjuggler_layout.xml; exec bash\"";
