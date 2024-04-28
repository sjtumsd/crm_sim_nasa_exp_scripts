// =============================================================================
// Author: Wei Hu
// Email:  weihu@sjtu.edu.cn
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;

// Physical properties of terrain particles
double iniSpacing;
double kernelLength;
double density;
double slope_angle;
double total_mass;

// output directories and settings
std::string out_dir = "/root/sbel/outputs/FSI_Viper_RealSlope_SlopeAngle_";

// Dimension of the space domain
double bxDim = 6.0;
double byDim = 2.0;
double bzDim = 0.2;

// Rover initial location
ChVector<> init_loc( 1.0 - bxDim / 2.0, 0, bzDim + 0.25);

// Simulation time and stepsize
double total_time = 20.0;
double dT;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 1;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 1;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/viper/obj/viper_wheel.obj";

// wheel specifics
double wheel_radius = 0.25;
double wheel_wide = 0.2;
double grouser_height = 0.025;
double grouser_wide = 0.005;
int grouser_num = 24;
double wheel_AngVel = 0.8;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

// Pointer to store the VIPER driver
std::shared_ptr<ViperSpeedDriver> driver;

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Forward declaration of helper functions
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime);
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);

int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Use JSON file to set the FSI parameters
    std::string inputJson = "/root/sbel/json/demo_ROBOT_Viper_RealSlope.json";
    if (argc == 4) {
        total_mass = std::stod(argv[1]);
        slope_angle = std::stod(argv[2]) / 180.0 * CH_C_PI;
        wheel_AngVel = std::stod(argv[3]);
        out_dir = out_dir + std::to_string(std::stoi(argv[2])) + "/";
    } else if (argc != 4) {
        std::cout << "usage: ./demo_ROBOT_Viper_RealSlope <total_mass> <slope_angle> <wheel_angVel>" << std::endl;
        return 1;
    }

    // Create oputput directories
    if (!filesystem::create_subdirectory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    sysFSI.ReadParametersFromFile(inputJson);

    double gravity_G = sysFSI.Get_G_acc().z();
    ChVector<> gravity = ChVector<>(gravity_G * sin(slope_angle), 0, gravity_G * cos(slope_angle));
    sysMBS.Set_G_acc(gravity);
    sysFSI.Set_G_acc(gravity);

    // Get the simulation stepsize
    dT = sysFSI.GetStepSize();

    // Get the initial particle spacing
    iniSpacing = sysFSI.GetInitialSpacing();

    // Get the SPH kernel length
    kernelLength = sysFSI.GetKernelLength();

    // // Set the initial particle spacing
    // sysFSI.SetInitialSpacing(iniSpacing);

    // // Set the SPH kernel length
    // sysFSI.SetKernelLength(kernelLength);

    // // Set the terrain density
    // sysFSI.SetDensity(density);

    // // Set the simulation stepsize
    // sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Set rigid body boundary condition
    sysFSI.SetRigidBodyBC(BceVersion::ADAMI);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(0.0);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    ChVector<> cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 2, byDim / 2  + 0.5 * iniSpacing, bzDim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(iniSpacing);
    ChVector<> boxCenter(0, 0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                              ChVector<>(0),         // initial velocity
                              ChVector<>(-pre_ini),  // tauxxyyzz
                              ChVector<>(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Viper on SPH terrain");
        fsi_vis.SetCameraPosition(ChVector<>(0, -3 * byDim, bzDim), ChVector<>(0, 0, 0));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.EnableRigidBodyMarkers(false);
        fsi_vis.AttachSystem(&sysMBS);
        fsi_vis.Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    // Get the chassis of the rover
    auto body = sysMBS.Get_bodylist()[1];

    ChTimer<> timer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl; 

        rover->Update();

        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPos_dt() << std::endl;
        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPos_dt() << std::endl;
            if (current_step % output_steps == 0) {
                sysFSI.PrintParticleToFile(out_dir + "/particles");
                sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
                SaveParaViewFiles(sysFSI, sysMBS, time);
            }
        }

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output)
        ofile.close();

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI) {
    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector<>(0, 0, 0));
    box->SetBodyFixed(true);
    sysMBS.Add(box);

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddContainerBCE(box, ChFrame<>(), ChVector<>(bxDim, byDim, 2 * bzDim), ChVector<int>(2, 0, -1));

    driver = chrono_types::make_shared<ViperSpeedDriver>(0.1, wheel_AngVel);
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // // Create the wheel's BCE particles
    // auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    // double scale_ratio = 1.0;
    // trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    // trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    // trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // std::vector<ChVector<>> BCE_wheel;
    // sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);

    // Set the rover mass to a user mass
    for (int i = 0; i < 17; i++) {
        double mass_scale = total_mass / 440.0;
        auto viper_part = sysMBS.Get_bodylist()[i+1];
        double part_mass = viper_part->GetMass();
        ChVector<> part_inertia = viper_part->GetInertiaXX();
        viper_part->SetMass(part_mass * mass_scale);
        viper_part->SetInertiaXX(part_inertia * mass_scale);
    }

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        // if (i == 0 || i == 2) {
        //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI)), true);
        // } else {
        //     sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        // }
        double inner_radius = wheel_radius-grouser_height;
        sysFSI.AddWheelBCE_Grouser(wheel_body, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, 
            grouser_height, grouser_wide, grouser_num, kernelLength, false);
    }

    {
        // Create the chassis of the test rig
        auto chassis = chrono_types::make_shared<ChBody>();
        chassis->SetMass(10.0);
        chassis->SetPos(init_loc);
        chassis->SetCollide(false);
        chassis->SetBodyFixed(false);

        // Add geometry of the chassis.
        chassis->GetCollisionModel()->ClearModel();
        chrono::utils::AddBoxGeometry(chassis.get(), 
            CustomWheelMaterial(ChContactMethod::NSC), ChVector<>(0.1, 0.1, 0.1), ChVector<>(0, 0, 0));
        chassis->GetCollisionModel()->BuildModel();
        sysMBS.AddBody(chassis);

        // // Create the axle
        // auto axle = chrono_types::make_shared<ChBody>();
        // axle->SetMass(10.0);
        // axle->SetPos(init_loc + ChVector<>(0, 0, 1));
        // axle->SetCollide(false);
        // axle->SetBodyFixed(false);

        // // Add geometry of the axle.
        // axle->GetCollisionModel()->ClearModel();
        // chrono::utils::AddSphereGeometry(axle.get(), CustomWheelMaterial(ChContactMethod::NSC), 0.5, ChVector<>(0, 0, 0));
        // axle->GetCollisionModel()->BuildModel();
        // sysMBS.AddBody(axle);

        // Connect the chassis to the containing bin (ground) through a translational joint and create a linear actuator.
        auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic1->Initialize(box, chassis, ChCoordsys<>(chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));
        prismatic1->SetName("prismatic_chassis_ground");
        sysMBS.AddLink(prismatic1);

        // auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, wheel_vel);
        // actuator->Initialize(box, chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
        //                     ChCoordsys<>(chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));
        // actuator->SetName("actuator");
        // actuator->SetDistanceOffset(1);
        // actuator->SetActuatorFunction(actuator_fun);
        // sysMBS.AddLink(actuator);

        // Connect the axle to the chassis through a vertical translational joint.
        auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
        auto rover_body = rover->GetChassis()->GetBody();
        prismatic2->Initialize(chassis, rover_body, ChCoordsys<>(chassis->GetPos(), QUNIT));
        prismatic2->SetName("prismatic_rover_chassis");
        sysMBS.AddLink(prismatic2);

        // // Connect the rover body to the axle through a engine joint.
        // auto lock_link = chrono_types::make_shared<ChLinkLockLock>();
        // auto rover_body = rover->GetChassis()->GetBody();
        // lock_link->SetName("rover_axle_lock");
        // lock_link->Initialize(axle, rover_body, ChCoordsys<>(chassis->GetPos(), QUNIT));
        // sysMBS.AddLink(lock_link);
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime) {
    std::string rover_dir = out_dir + "/rover";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // save rigid body position and rotation
    for (int i = 1; i < sysMBS.Get_bodylist().size(); i++) {
        auto body = sysMBS.Get_bodylist()[i];
        ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> pos = ref_frame.GetPos();
        ChQuaternion<> rot = ref_frame.GetRot();
        ChVector<> vel = body->GetPos_dt();

        std::string delim = ",";
        filename = rover_dir + "/body_pos_rot_vel" + std::to_string(i) + ".csv";
        std::ofstream file;
        if (sysMBS.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
        }

        file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
             << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim << vel.y()
             << delim << vel.z() << std::endl;

        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << mTime << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}
