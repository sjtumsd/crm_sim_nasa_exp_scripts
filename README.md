# Scripts and Instructions
Clone the scripts and follow the below instructions to reproduce the results we published in the paper.

PS: We ran simulations and genereated all the results on Ubuntu 20.04 with CUDA 11.8 and GCC 9.3.0.


## Step 1 - Clone Chrono and scripts for the paper, then build Chrono software
Creat a work directory in your home directory like below and go to this folder: 

```/home/weihu/research/00_CRM_NASA_SIM/```

Go to this work directory, clone the chrono repository and checkout to release 8.0 version: 

```git clone https://github.com/projectchrono/chrono.git --recursive -b release/8.0```

Clone the scripts repository for the paper: 

```git clone https://github.com/sjtumsd/crm_sim_nasa_exp_scripts.git```

Make some necessary changes to the chrono source files by replacing them with the provided files : 

```cp crm_sim_nasa_exp_scripts/chrono_source_files/Ch* chrono/src/chrono_fsi/```

```cp crm_sim_nasa_exp_scripts/chrono_source_files/Viper.cpp chrono/src/chrono_models/robot/viper/```

Make a build directory to build chrono software:

```mkdir chrono_build```

```cd chrono_build```

Build chrono in this directody with ```FSI Module``` and ```Vehicle Module``` enabled


## Step 2 - Build and run the single wheel simulation with VV-mode
Go to the single wheel VV-mode scripts folder 

```cd crm_sim_nasa_exp_scripts\demos\single_wheel_vv_mode```

Configure the single wheel simulation 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=/home/weihu/research/00_CRM_NASA_SIM/chrono_build/cmake```

Build the simulation

```make```

Run the simulation

```./runSimulation.sh```


## Step 3 - Build and run the single wheel simulation with Real slope-mode
Go to the single wheel Real slope-mode scripts folder 

```cd crm_sim_nasa_exp_scripts\demos\single_wheel_real_slope_mode```

Configure the single wheel simulation 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=/home/weihu/research/00_CRM_NASA_SIM/chrono_build/cmake```

Build the simulation

```make```

Run the simulation

```./runSimulation.sh```

## Step 4 - Build and run the full Viper rover simulation
Go to the full Viper rover scripts folder 

```cd crm_sim_nasa_exp_scripts\demos\viper_real_slope```

Configure the full Viper rover simulation 

```cmake . -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=/home/weihu/research/00_CRM_NASA_SIM/chrono_build/cmake```

Build the simulation

```make```

Run the simulation

```./runSimulation.sh```