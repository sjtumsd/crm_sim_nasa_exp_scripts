# Scripts and Instructions
Clone the scripts and follow the below instructions to reproduce the results we published in the paper.

We ran simulations and generated all the results on Ubuntu 20.04 with CUDA 11.8 and GCC 9.3.0. 

These simulations took 3-4 days on a workstation with Intel 14900K CPU and NVIDIA RTX 4090 GPU.


## Step 1 - Clone Chrono and scripts for the paper, then build Chrono software
Create a work directory in your home directory like below: 

```~/00_CRM_NASA_SIM```

Go to this work directory, clone the Chrono repository and checkout to release 8.0 version: 

```git clone https://github.com/projectchrono/chrono.git --recursive -b release/8.0```

Clone the scripts repository for the paper: 

```git clone https://github.com/sjtumsd/crm_sim_nasa_exp_scripts.git```

Make some necessary changes to the Chrono source files by replacing them with the provided files: 

```cp crm_sim_nasa_exp_scripts/chrono_source_files/Ch* chrono/src/chrono_fsi/```

```cp crm_sim_nasa_exp_scripts/chrono_source_files/Viper.cpp chrono/src/chrono_models/robot/viper/```

Create a directory to build Chrono software and go to it:

```mkdir chrono_build```

```cd chrono_build```

Build Chrono in this directory with ```FSI MODULE``` ON, ```VEHICLE MODULE``` ON, and ```USE_FSI_DOUBLE``` OFF


## Step 2 - Build and run the single wheel simulation with VV-mode
Go to the single wheel VV-mode scripts folder: 

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts/demos/single_wheel_vv_mode```

Configure the single wheel simulation: 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=../../../chrono_build/cmake```

Build the simulation:

```make```

Run the simulation:

```sudo chmod +x runSimulation.sh```

```./runSimulation.sh```


## Step 3 - Build and run the single wheel simulation with Real slope-mode
Go to the single wheel Real slope-mode scripts folder:

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts/demos/single_wheel_real_slope_mode```

Configure the single wheel simulation: 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=../../../chrono_build/cmake```

Build the simulation:

```make```

Run the simulation:

```sudo chmod +x runSimulation.sh```

```./runSimulation.sh```

## Step 4 - Build and run the full Viper rover simulation
Go to the full Viper rover scripts folder 

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts/demos/viper_real_slope```

Configure the full Viper rover simulation 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=../../../chrono_build/cmake```

Build the simulation:

```make```

Run the simulation:

```sudo chmod +x runSimulation.sh```

```sudo chmod +x runSimulationConclusion.sh```

```./runSimulation.sh```

```./runSimulationConclusion.sh```

## Step 5 - Generate all images in the paper
Go to below directory:

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts```

Download Blender package

```wget https://mirror.clarkson.edu/blender/release/Blender3.3/blender-3.3.1-linux-x64.tar.xz```

```tar -xvf blender-3.3.1-linux-x64.tar.xz```


Go to below directory:

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts/images```

Generate all images by running below script:

```sudo chmod +x generateFigures.sh```

```./generateFigures.sh```

All the images will be in:

```~/00_CRM_NASA_SIM/crm_sim_nasa_exp_scripts/images/figure```