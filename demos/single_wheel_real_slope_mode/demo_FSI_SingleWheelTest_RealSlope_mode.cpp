// =============================================================================
// Author: Wei Hu
// Email:  weihu@sjtu.edu.cn
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/core/ChTimer.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;

// Your work directory
std::string work_dir = "../..";

// Physical properties of terrain particles
double iniSpacing;
double kernelLength;
double density;
double slope_angle; // Terrain slope 

// Dimension of the terrain container
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.2 + smalldis;

// Size of the wheel
double wheel_radius = 0.25;
double wheel_wide = 0.2;
double grouser_height = 0.025;
double grouser_wide = 0.005;
int grouser_num = 24;

double wheel_slip = 0.0;
double wheel_vel = 0.2;
double wheel_AngVel = 0.8;
double total_mass = 17.5;
std::string wheel_obj = work_dir + "/obj/viper_wheel.obj";

// Initial Position of wheel
ChVector<> wheel_IniPos(-bxDim / 2 + wheel_radius * 2.0, 0.0, wheel_radius + bzDim + 0.01);
ChVector<> wheel_IniVel(0.0, 0.0, 0.0);

// Simulation time and stepsize
double total_time = 20.0;
double dT;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 1;

// Output directories and settings
std::string out_dir = "outputs";

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 1;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

//------------------------------------------------------------------
// Function to save wheel to Paraview VTK files
//------------------------------------------------------------------
void WriteWheelVTK(const std::string& filename,
                   ChTriangleMeshConnected& mesh,
                   const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.getCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.getCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.getIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.getIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);
    
    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    sysMBS.AddBody(ground);

    ground->GetCollisionModel()->ClearModel();
    chrono::utils::AddBoxContainer(ground, cmaterial, ChFrame<>(), ChVector<>(bxDim, byDim, bzDim), 0.1,
                                   ChVector<int>(0, 0, -1), false);
    ground->GetCollisionModel()->BuildModel();
    ground->SetCollide(true);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddContainerBCE(ground, ChFrame<>(), ChVector<>(bxDim, byDim, 2 * bzDim), ChVector<int>(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(wheel_obj, false, true);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.0;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector<>(0.0, 0.0, 0.0);

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = Q_from_Euler123(ChVector<double>(0, 0, 0));

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    wheel->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPos_dt(wheel_IniVel);
    wheel->SetWvel_loc(ChVector<>(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    wheel->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    sysMBS.AddBody(wheel);

    wheel->SetBodyFixed(false);
    wheel->GetCollisionModel()->ClearModel();
    wheel->GetCollisionModel()->AddTriangleMesh(cmaterial, trimesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
    wheel->GetCollisionModel()->BuildModel();
    wheel->SetCollide(false);

    // Add this body to the FSI system
    // std::vector<ChVector<>> BCE_wheel;
    // sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);
    // sysFSI.AddPointsBCE(wheel, BCE_wheel, ChFrame<>(), true);
    // sysFSI.AddFsiBody(wheel);

    // Add BCE particles attached on the cylinder into FSI system
    double inner_radius = wheel_radius-grouser_height;
    sysFSI.AddWheelBCE_Grouser(wheel, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, 
        grouser_height, grouser_wide, grouser_num, kernelLength, false);
    sysFSI.AddFsiBody(wheel);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(2.0);
    chassis->SetPos(wheel->GetPos());
    chassis->SetCollide(false);
    chassis->SetBodyFixed(false);

    // Add geometry of the chassis.
    chassis->GetCollisionModel()->ClearModel();
    chrono::utils::AddBoxGeometry(chassis.get(), cmaterial, ChVector<>(0.1, 0.1, 0.1), ChVector<>(0, 0, 0));
    chassis->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->SetCollide(false);
    axle->SetBodyFixed(false);

    // Add geometry of the axle.
    axle->GetCollisionModel()->ClearModel();
    chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5, ChVector<>(0, 0, 0));
    axle->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChCoordsys<>(chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // double velocity = wheel_vel; //wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    // auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, velocity);

    // actuator->Initialize(ground, chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
    //                      ChCoordsys<>(chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));
    // actuator->SetName("actuator");
    // actuator->SetDistanceOffset(1);
    // actuator->SetActuatorFunction(actuator_fun);
    // sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChCoordsys<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), 
        chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // The path to the Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysFSI.SetVerbose(verbose_fsi);

    // Use JSON file to set the FSI parameters
    std::string inputJson = work_dir + "/json";
    if (argc == 7) {
        total_mass = std::stod(argv[1]);
        slope_angle = std::stod(argv[2]) / 180.0 * CH_C_PI;
        wheel_AngVel = std::stod(argv[3]);
        
        if (std::stoi(argv[4]) == 1){
            inputJson = inputJson + "/GRC1";
            out_dir = out_dir + "/GRC1";
        } else if (std::stoi(argv[4]) == 3){
            inputJson = inputJson + "/GRC3";
            out_dir = out_dir + "/GRC3";
        }
        if (std::stoi(argv[5]) == 1){
            inputJson = inputJson + "/Earth";
            out_dir = out_dir + "/Earth";
        } else if (std::stoi(argv[5]) == 2){
            inputJson = inputJson + "/Moon";
            out_dir = out_dir + "/Moon";
        }
        out_dir = out_dir + "/omega" + argv[3];
        if (std::stoi(argv[6]) == 1){
            inputJson = inputJson + "/1.json";
            out_dir = out_dir + "/soil1_";
        } else if (std::stoi(argv[6]) == 2){
            inputJson = inputJson + "/2.json";
            out_dir = out_dir + "/soil2_";
        } else if (std::stoi(argv[6]) == 3){
            inputJson = inputJson + "/3.json";
            out_dir = out_dir + "/soil3_";
        }
        out_dir = out_dir + "slope" + std::to_string(std::stoi(argv[2])) + "deg/";
    } else if (argc != 7) {
        std::cout << "usage: ./demo_FSI_SingleWheelTest_RealSlope_mode <total_mass> <slope_angle> <wheel_angVel>" << std::endl;
        std::cout << "or to use default input parameters ./demo_FSI_SingleWheelTest_RealSlope_mode " << std::endl;
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
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
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

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Set rigid body boundary condition
    sysFSI.SetRigidBodyBC(BceVersion::ADAMI);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(0.0);

    // Setup the SPH method
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector<> cMin(-bxDim / 2 * 10, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 10, byDim / 2 + 0.5 * iniSpacing, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    ChVector<> boxCenter(0.0, 0.0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    sysFSI.AddBoxSPH(boxCenter, boxHalfDim);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.Get_bodylist()[1];
    ChVector<> force = actuator->Get_react_force();
    ChVector<> torque = motor->Get_react_torque();
    ChVector<> w_pos = wheel->GetPos();
    ChVector<> w_vel = wheel->GetPos_dt();
    ChVector<> angvel = wheel->GetWvel_loc();

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(wheel_obj, false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI single wheel demo");
        fsi_vis.SetCameraPosition(ChVector<>(0, -5 * byDim, 5 * bzDim), ChVector<>(0, 0, 0));
        fsi_vis.SetCameraMoveScale(0.05f);
        fsi_vis.EnableBoundaryMarkers(true);
        fsi_vis.Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    ChTimer<> timer;
    timer.start();
    while (time < total_time) {
        // Get the infomation of the wheel
        force = actuator->Get_react_force();
        torque = motor->Get_react_torque();
        w_pos = wheel->GetPos();
        w_vel = wheel->GetPos_dt();
        angvel = wheel->GetWvel_loc();

        if (verbose) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        if (output) {
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
        }

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
            WriteWheelVTK(filename, wheel_mesh, wheel->GetFrame_REF_to_abs());
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}
