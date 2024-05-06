#! /bin/bash 

echo "Start to run the simulation"

echo "Earth gravity, GRC3, soil 1, omega=0.8"
./demo_ROBOT_Viper_RealSlope 73.0 0  0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 5  0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 10 0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 15 0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 20 0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 25 0.8 3 1 1
./demo_ROBOT_Viper_RealSlope 73.0 30 0.8 3 1 1

echo "Earth gravity, GRC3, soil 2, omega=0.8"
./demo_ROBOT_Viper_RealSlope 73.0 0  0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 5  0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 10 0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 15 0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 20 0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 25 0.8 3 1 2
./demo_ROBOT_Viper_RealSlope 73.0 30 0.8 3 1 2

echo "Earth gravity, GRC3, soil 3, omega=0.8"
./demo_ROBOT_Viper_RealSlope 73.0 0  0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 5  0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 10 0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 15 0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 20 0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 25 0.8 3 1 3
./demo_ROBOT_Viper_RealSlope 73.0 30 0.8 3 1 3

echo "Moon gravity, GRC3, soil 1, omega=0.8"
./demo_ROBOT_Viper_RealSlope 440.0 0  0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 5  0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 10 0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 15 0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 20 0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 25 0.8 3 2 1
./demo_ROBOT_Viper_RealSlope 440.0 30 0.8 3 2 1

echo "Moon gravity, GRC3, soil 2, omega=0.8"
./demo_ROBOT_Viper_RealSlope 440.0 0  0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 5  0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 10 0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 15 0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 20 0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 25 0.8 3 2 2
./demo_ROBOT_Viper_RealSlope 440.0 30 0.8 3 2 2

echo "Moon gravity, GRC3, soil 3, omega=0.8"
./demo_ROBOT_Viper_RealSlope 440.0 0  0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 5  0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 10 0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 15 0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 20 0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 25 0.8 3 2 3
./demo_ROBOT_Viper_RealSlope 440.0 30 0.8 3 2 3

echo "Finished the simulation"