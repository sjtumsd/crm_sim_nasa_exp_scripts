# workshop_demo
The Chrono demos are containerized with Docker. Here are some instructions how to set up container on your machine so that you can successfully run the demo.

## Building an image
### Method 1 ---- Build from scratch 
Open a terminal in your machine then run 

```git clone https://github.com/uwsbel/workshop_demo.git && cd workshop_demo```

Once getting into folder, running the following to build docker image from Dockerfile:

``` docker build -t <img_name> . ```

Notice that you can put a tag name for the image you build by using flag -t. For example,

``` docker build -t uwsbel/demo . ```

builds an image named uwsbel/demo.


### Method 2 ---- Pulling Our Docker Image

Pulling the Docker image by running:

```docker pull uwsbel/demo```

## Running a container based on an image

After building the image using either method, create and get into a container using the command:

```docker run -it --gpus all -v <dir_to_store_data>:/root/sbel/outputs -v <dir_to_json_inputs>:/root/sbel/json uwsbel/demo ```

Note: 
- ```<dir_to_store_data>``` is the host machine directory where you want to store the output data from the demos. 
- ```<dir_to_json_inputs>``` is the host machine directory of json inputs. 
- ```/root/sbel/outputs``` is the output directory in the container.
- ```/root/sbel/json``` is the json input directory in the container.

```<dir_to_store_data>``` and ```<dir_to_json_inputs>``` depends on your operating system and work directory.

Windows user will have something like:```C:\Users\SBEL\demo_output\``` and ``` C:\Users\SBEL\workshop_demo\json```.

Linux user will have something like: ```/home/harry/workshop_demo/outputs/``` and ```/home/harry/workshop_demo/json/```

## Run the demo
Single wheel test under VV mode

```./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3```

- 17.5 is the mass of the wheel
- 0.3 is slip ratio that would like to enforce
- 3 is just an ID for this slip

Sinle wheel test under real slope mode

```./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 15 0.8```

- 17.5 is the mass of the wheel
- 15 is the slope angle of the terrain
- 0.8 is the wheel angular velocity

Full VIPER rover under real slope

```./demo_ROBOT_Viper_RealSlope 73.0 15 0.8```

- 73.0 is the mass of the rover
- 15 is the slope angle of the terrain
- 0.8 is the wheel angular velocity

Curiosity rover on uphill and downhill

```./demo_ROBOT_Curiosity_Uphill 200.0 1.0 1```

- 200.0 is the mass of the rover
- 1.0 is the height of the terrain
- 1 is just an ID for this simulation

## Render the VIPER rover results using Blender
- Go to one of the full VIPER rover result folder, e.g /FSI_Viper_RealSlope_SlopeAngle_15
- Copy the script "blender_viper_render.py" and the obj file folder "obj_for_render" to the above result folder
- Render an image using below command (7 is the frame number of the output image)

```blender --background --python ./blender_viper_render.py 7```

## Render the Curiosity rover results using Blender
- Go to one of the Curiosity rover result folder, e.g /FSI_Curiosity_Uphill_1
- Copy the script "blender_curiosity_render.py" and the obj file folder "obj_for_render" to the above result folder
- Render an image using below command (15 is the frame number of the output image)

```blender --background --python ./blender_curiosity_render.py 15```
