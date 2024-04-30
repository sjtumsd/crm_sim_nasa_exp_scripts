#! /bin/bash 
echo "Start to run the simulation"

./demo_FSI_SingleWheelTest_VV_mode 17.5 0.0 0 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.1 1 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.2 2 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.4 4 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.5 5 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.6 6 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.7 7 3 1 1
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.8 8 3 1 1

./demo_FSI_SingleWheelTest_VV_mode 17.5 0.0 0 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.1 1 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.2 2 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.4 4 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.5 5 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.6 6 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.7 7 3 1 3
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.8 8 3 1 3

echo "Finished the simulation"