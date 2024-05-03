#! /bin/bash 
echo "Start Running Demo"
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.0 0
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.1 1
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.2 2
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.4 4
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.5 5
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.6 6
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.7 7
cd /root/sbel/demos/single_wheel_vv_mode/build/ && ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.8 8

cp /root/sbel/demos/single_wheel_vv_mode/slope_slip_wheel_vv_mode.py /root/sbel/outputs/
cd /root/sbel/outputs/ && python3 slope_slip_wheel_vv_mode.py
echo "Finished single wheel test under VV mode"

cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 0 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 5 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 10 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 15 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 20 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 25 0.8
cd /root/sbel/demos/single_wheel_real_slope_mode/build/ && ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 30 0.8

cp /root/sbel/demos/single_wheel_real_slope_mode/slope_slip_wheel_realSlope_mode.py /root/sbel/outputs/
cd /root/sbel/outputs/ && python3 slope_slip_wheel_realSlope_mode.py
echo "Finished single wheel test under real slope mode"

echo "Start Running VIPER and Curiosity demo"
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 0 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 5 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 10 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 15 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 20 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 25 0.8
cd /root/sbel/demos/viper_real_slope/build/ && ./demo_ROBOT_Viper_RealSlope 73.0 30 0.8
cd /root/sbel//demos/curiosity_uphill/build/ && ./demo_ROBOT_Curiosity_Uphill 200.0 1.0 1
echo "Finished VIPER and Curiosity demos"

cp /root/sbel/demos/viper_real_slope/slope_slip_full_rover.py /root/sbel/outputs/
cd /root/sbel/outputs/ && python3 slope_slip_full_rover.py

cp /root/sbel/demos/viper_real_slope/blender_viper_render.py /root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_0/
cp -r /root/sbel/demos/viper_real_slope/obj_for_render/ /root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_0/
for i in {0..19}
do
        cd /root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_0/ && blender --background --python ./blender_viper_render.py $i
done
cd /root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_0/ && ffmpeg -r 1 -f image2 -s 1920x1080 -i %d.png -vframes 20 -vcodec libx264 -crf 1 -pix_fmt yuv420p viper_animation.mp4
cp /root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_0/viper_animation.mp4 /root/sbel/outputs/
echo "Finished VIPER postprocessing"

cp /root/sbel/demos/curiosity_uphill/blender_curiosity_render.py /root/sbel/outputs/FSI_Curiosity_Uphill_1/
cp -r /root/sbel/demos/curiosity_uphill/obj_for_render/ /root/sbel/outputs/FSI_Curiosity_Uphill_1/
for i in {0..24}
do
        cd /root/sbel/outputs/FSI_Curiosity_Uphill_1/ && blender --background --python ./blender_curiosity_render.py $i
done
cd /root/sbel/outputs/FSI_Curiosity_Uphill_1/ && ffmpeg -r 1 -f image2 -s 1920x1080 -i %d.png -vframes 25 -vcodec libx264 -crf 1 -pix_fmt yuv420p curiosity_animation.mp4
cp /root/sbel/outputs/FSI_Curiosity_Uphill_1/curiosity_animation.mp4 /root/sbel/outputs/
echo "Finished Curiosity postprocessing"
