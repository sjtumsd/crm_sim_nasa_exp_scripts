// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Wei Hu
// Demo to show usage of Curiosity rover models on SPH granular terrain
// This demo uses a plug-in Curiosity rover model from chrono::models
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;

// Physical properties of terrain particles
double iniSpacing;
double kernelLength;
double density;
double slope_angle;
double hill_height;
double total_mass;
double part_density = 2879;

// output directories and settings
std::string out_dir = "/root/sbel/outputs/FSI_Curiosity_Uphill_";

// Dimension of the space domain
double bxDim = 6.0;
double byDim = 3.0;
double bzDim = 1.0;

// Length of the top of the heap
double L_top = 1.0; 

// Rover initial location
ChVector<> init_loc(bxDim / 2.0 + 1.3, 0, 0.05);

// Simulation time and stepsize
double total_time = 25.0;
double dT;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
bool save_obj = false;  // if true, save as Wavefront OBJ; if false, save as VTK
int out_fps = 1;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 1;

// wheel specifics
double wheel_radius = 0.25;
double wheel_wide = 0.2;
double grouser_height = 0.025;
double grouser_wide = 0.005;
int grouser_num = 24;
double wheel_AngVel = 0.5 * CH_C_PI;

std::vector<ChVector<>> BCE_Wall;

int motor_type = 1; // 1 means constant rotation speed, 2 means constant torque
double motor_F = 0.5; // if motor_type==1, this means the rotation speed, if motor_type==2, this means the torque
int control_on = 0;
double motor_ang_v_min;
double motor_ang_v_max;
double motor_acc_rate;
double slip_base;

// Motor on each wheel
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_lf; 
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_rf; 
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_lm; 
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_rm; 
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_lb; 
std::shared_ptr<ChLinkMotorRotationSpeed> link_motor_speed_rb; 

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
    std::string inputJson = "/root/sbel/json/demo_ROBOT_Curiosity_Uphill.json";
    if (argc == 4) {
        total_mass = std::stod(argv[1]);
        hill_height = std::stod(argv[2]);
        out_dir = out_dir + std::to_string(std::stoi(argv[3])) + "/";
        bzDim = hill_height;
    } else if (argc != 4) {
        std::cout << "usage: ./demo_ROBOT_Curiosity_Uphill <total_mass> <hill_height>" << std::endl;
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
    ChVector<> gravity = ChVector<>(0, 0, gravity_G);
    sysMBS.Set_G_acc(gravity * (total_mass / 900.0));

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
    ChVector<> cMin(-bxDim / 2 * 3, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 2);
    ChVector<> cMax( bxDim / 2 * 3,  byDim / 2 + 0.5 * iniSpacing,  bzDim * 2);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(iniSpacing);
    ChVector<> boxCenter(0, 0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2.0 * 2.1, byDim / 2, bzDim / 2 + 15 * iniSpacing);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double x_ini = points[i].x();
        double y_ini = points[i].y();
        double z_ini = points[i].z();
        double hL_top = L_top / 2.0; // half length of the top of the heap
        double z_max;
        if(abs(x_ini) > hL_top + 0.001*iniSpacing && abs(x_ini) < bxDim / 2){
            z_max = bzDim * (abs(bxDim / 2) - abs(x_ini)) / (abs(bxDim / 2) - hL_top) + 0.001*iniSpacing;
        }
        else if(abs(x_ini) < hL_top + 0.001*iniSpacing){
            z_max = bzDim + 0.001*iniSpacing;
        }
        else{
            z_max = 0.0;
        }
        double z_mid = z_max - 10*iniSpacing;
        double z_low = z_max - 15*iniSpacing;
        if(z_ini < z_max){
            if(z_ini > z_mid){
                if(x_ini < bxDim  && x_ini > -bxDim ){
                    sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                                        ChVector<>(0),  // initial velocity
                                        ChVector<>(0),  // tauxxyyzz
                                        ChVector<>(0)   // tauxyxzyz
                    );
                }
            }
            else if(z_ini > z_low){
                BCE_Wall.push_back(points[i]);
            }
        }
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    double time = 0.0;
    int current_step = 0;

    // Get the chassis of the rover
    auto body = sysMBS.Get_bodylist()[1];

    ChTimer<> timer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl; 

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

        ChVector<> vel = body->GetPos_dt();
        body->SetPos_dt(ChVector<>(vel.x(), 0.0, vel.z()));

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
    // My material surface for contact
    auto myMat = CustomWheelMaterial(ChContactMethod::NSC);

    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 0.02, 1000, false, false, myMat);
    box->SetPos(ChVector<>(0, 0, 0));
    box->SetBodyFixed(true);
    sysMBS.Add(box);

    // Fluid-Solid Coupling at the walls via BCE particles
    // sysFSI.AddContainerBCE(box, ChFrame<>(), ChVector<>(bxDim, byDim, bzDim), ChVector<int>(0, 0, -1));
    sysFSI.AddPointsBCE(box, BCE_Wall, ChFrame<>(), false);


    // ============================================================================
    // ======================= Assemble the Curiosity =============================
    // ============================================================================
    double rover_mass = 0;
    // Create the Curiosity chassis
    {
        // load mesh from obj file
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "../obj_for_render/body.obj";
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));     // rotate the mesh if needed
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = part_density;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmass = 0.23;
        mcog = ChVector<>(0, 0, 0);
        minertia = ChMatrix33<>(1.0);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, -CH_C_PI/2));
        ChVector<> Body_pos = ChVector<>(init_loc.x(), 
                                         init_loc.y(),
                                         init_loc.z());
        ChVector<> Body_vel = ChVector<>(-0.0, 0.0, 0.0);

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        std::cout << "\n" << "The mass of the rover body is " << mmass * mdensity << "\n" << std::endl;
        rover_mass = rover_mass + mmass * mdensity;
        Body->SetMass(mmass * mdensity);//mmass * mdensity
        Body->SetInertiaXX(mdensity * principal_I);
        // Body->SetPos(Body_pos);
        Body->SetPos_dt(Body_vel);
        // Body->SetRot(QUNIT);
        
		// Set the absolute position of the body:
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
        sysMBS.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(myMat, mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(false);
    }

    // Create wheels
    ChVector<> wheel_rel_pos_lf = ChVector<>(-1.095, -1.063, 0.249);
    ChVector<> wheel_rel_pos_rf = ChVector<>(-1.095,  1.063, 0.249);
    ChVector<> wheel_rel_pos_lm = ChVector<>( 0.089, -1.194, 0.249);
    ChVector<> wheel_rel_pos_rm = ChVector<>( 0.089,  1.194, 0.249);
    ChVector<> wheel_rel_pos_lb = ChVector<>( 1.163, -1.063, 0.249);
    ChVector<> wheel_rel_pos_rb = ChVector<>( 1.163,  1.063, 0.249);
    {
    for (int i = 0; i < 6; i++) {
        // load mesh from obj file
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "../obj_for_render/wheel.obj";
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = part_density;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        double RotAng;
        ChVector<> Body_Rel_pos;
        if(i==0){Body_Rel_pos = wheel_rel_pos_lf; RotAng = 0.0;}
        if(i==1){Body_Rel_pos = wheel_rel_pos_rf; RotAng = CH_C_PI;}
        if(i==2){Body_Rel_pos = wheel_rel_pos_lm; RotAng = 0.0;}
        if(i==3){Body_Rel_pos = wheel_rel_pos_rm; RotAng = CH_C_PI;}
        if(i==4){Body_Rel_pos = wheel_rel_pos_lb; RotAng = 0.0;}
        if(i==5){Body_Rel_pos = wheel_rel_pos_rb; RotAng = CH_C_PI;}
        ChVector<> Body_pos = ChVector<>(init_loc.x(), 
                                         init_loc.y(),
                                         init_loc.z()) + Body_Rel_pos;
        ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(0, 0, RotAng));
        ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        double wheel_mass = 2.5;
        double wheel_scale = wheel_mass / (mmass * mdensity);
        std::cout << "\n" << "The mass of the rover wheel is " << mmass * mdensity * wheel_scale << "\n" << std::endl;
        rover_mass = rover_mass + mmass * mdensity * wheel_scale;
        Body->SetMass(mmass * mdensity);
        Body->SetInertiaXX(mdensity * principal_I * wheel_scale);
        // Body->SetPos(Body_pos);
        Body->SetPos_dt(Body_vel);
        // Body->SetRot(QUNIT);
        
		// Set the absolute position of the body:
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
        sysMBS.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(myMat, mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(true);

        // Add this body to the FSI system
        sysFSI.AddFsiBody(Body);
        double inner_radius = wheel_radius - grouser_height;
        sysFSI.AddWheelBCE_Grouser(Body, ChFrame<>(), inner_radius, wheel_wide - iniSpacing, 
            grouser_height, grouser_wide, grouser_num, kernelLength, false);
    }
    }

    // Create connecting rod
    ChVector<> cr_rel_pos_lf = ChVector<>(-0.214, -0.604, 0.8754);
    ChVector<> cr_rel_pos_rf = ChVector<>(-0.214,  0.604, 0.8754);
    ChVector<> cr_rel_pos_lb = ChVector<>( 0.54,  -0.845, 0.6433);
    ChVector<> cr_rel_pos_rb = ChVector<>( 0.54,   0.845, 0.6433);
    {
    for (int i = 0; i < 4; i++) {
        // load mesh from obj file
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path;
        if(i == 0){obj_path = "../obj_for_render/F_L.obj";}
        if(i == 1){obj_path = "../obj_for_render/F_R.obj";}
        if(i == 2){obj_path = "../obj_for_render/B_L.obj";}
        if(i == 3){obj_path = "../obj_for_render/B_R.obj";}
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = part_density;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0, -CH_C_PI/2));
        ChVector<> Body_Rel_pos;
        if(i==0){Body_Rel_pos = cr_rel_pos_lf; }
        if(i==1){Body_Rel_pos = cr_rel_pos_rf; }
        if(i==2){Body_Rel_pos = cr_rel_pos_lb; }
        if(i==3){Body_Rel_pos = cr_rel_pos_rb; }
        ChVector<> Body_pos = ChVector<>(init_loc.x(), 
                                         init_loc.y(),
                                         init_loc.z()) + Body_Rel_pos;
        ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        std::cout << "\n" << "The mass of the connecting rod is " << mmass * mdensity << "\n" << std::endl;
        rover_mass = rover_mass + mmass * mdensity;
        Body->SetMass(mmass * mdensity);
        Body->SetInertiaXX(mdensity * principal_I);
        // Body->SetPos(Body_pos);
        Body->SetPos_dt(Body_vel);
        // Body->SetRot(QUNIT);
        
		// Set the absolute position of the body:
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
        sysMBS.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(myMat, mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(false);
    }
    }

    // Create steering rod on wheels
    ChVector<> sr_rel_pos_lf = ChVector<>(-1.095, -1.063, 0.64);
    ChVector<> sr_rel_pos_rf = ChVector<>(-1.095,  1.063, 0.64);
    ChVector<> sr_rel_pos_lb = ChVector<>( 1.163, -1.063, 0.64);
    ChVector<> sr_rel_pos_rb = ChVector<>( 1.163,  1.063, 0.64);
    {
    for (int i = 0; i < 4; i++) {
        // load mesh from obj file
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path;
        if(i == 0){obj_path = "../obj_for_render/ster_front.obj";}
        if(i == 1){obj_path = "../obj_for_render/ster_front.obj";}
        if(i == 2){obj_path = "../obj_for_render/ster_back.obj";}
        if(i == 3){obj_path = "../obj_for_render/ster_back.obj";}
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = part_density;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> Body_rot;
        ChVector<> Body_Rel_pos;
        if(i==0){Body_Rel_pos = sr_rel_pos_lf; Body_rot = Q_from_Euler123(ChVector<double>(0, 0,-CH_C_PI/2)); }
        if(i==1){Body_Rel_pos = sr_rel_pos_rf; Body_rot = Q_from_Euler123(ChVector<double>(0, 0, CH_C_PI/2)); }
        if(i==2){Body_Rel_pos = sr_rel_pos_lb; Body_rot = Q_from_Euler123(ChVector<double>(0, 0,-CH_C_PI/2)); }
        if(i==3){Body_Rel_pos = sr_rel_pos_rb; Body_rot = Q_from_Euler123(ChVector<double>(0, 0, CH_C_PI/2)); }
        ChVector<> Body_pos = ChVector<>(init_loc.x(), 
                                         init_loc.y(),
                                         init_loc.z()) + Body_Rel_pos;
        ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        std::cout << "\n" << "The mass of the steering rod is " << mmass * mdensity << "\n" << std::endl;
        rover_mass = rover_mass + mmass * mdensity;
        Body->SetMass(mmass * mdensity);
        Body->SetInertiaXX(mdensity * principal_I);
        // Body->SetPos(Body_pos);
        Body->SetPos_dt(Body_vel);
        // Body->SetRot(QUNIT);
        
		// Set the absolute position of the body:
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
        sysMBS.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(myMat, mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(false);
    }
    }

    // Create top rod
    ChVector<> tr_rel_pos_l = ChVector<>(-0.214, -0.672, 1.144);
    ChVector<> tr_rel_pos_r = ChVector<>(-0.214,  0.672, 1.144);
    ChVector<> tr_rel_pos_t = ChVector<>( 0.142,  0.0,   1.172);
    {
    for (int i = 0; i < 3; i++) {
        // load mesh from obj file
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path;
        if(i == 0){obj_path = "../obj_for_render/bar_l.obj";}
        if(i == 1){obj_path = "../obj_for_render/bar_r.obj";}
        if(i == 2){obj_path = "../obj_for_render/balancer.obj";}
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = part_density;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> Body_rot;
        ChVector<> Body_Rel_pos;
        if(i==0){Body_Rel_pos = tr_rel_pos_l; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0,-CH_C_PI/2)); }
        if(i==1){Body_Rel_pos = tr_rel_pos_r; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0,-CH_C_PI/2)); }
        if(i==2){Body_Rel_pos = tr_rel_pos_t; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0,-CH_C_PI/2)); }
        ChVector<> Body_pos = ChVector<>(init_loc.x(), 
                                         init_loc.y(),
                                         init_loc.z()) + Body_Rel_pos;
        ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        std::cout << "\n" << "The mass of the top bar is " << mmass * mdensity << "\n" << std::endl;
        rover_mass = rover_mass + mmass * mdensity;
        Body->SetMass(mmass * mdensity);
        Body->SetInertiaXX(mdensity * principal_I);
        // Body->SetPos(Body_pos);
        Body->SetPos_dt(Body_vel);
        // Body->SetRot(QUNIT);
        
		// Set the absolute position of the body:
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
        sysMBS.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(myMat, mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(false);

        // auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        // masset_mesh->SetMesh(mmesh);
        // masset_mesh->SetBackfaceCull(true);
        // Body->AddAsset(masset_mesh);
    }
    }
    std::cout << "\n" << "The total mass of the Curiosity rover is " << rover_mass << "\n" << std::endl;

    // ============================================================================
    // ======================== constraints =======================================
    // ============================================================================
    // Create joint constraints for wheels
    for (int i = 0; i < 6; i++) {
        // pick up bodies and create links
        auto wheel = sysMBS.Get_bodylist()[i+2];
        auto body_connected_to_wheel = sysMBS.Get_bodylist()[0];
        
        // define a rotation of -90 degrees around x (z2y)
        ChQuaternion<> z2y;
        z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));

        // pick up relative position of the link 
        ChVector<> Link_pos;
        ChVector<> Rover_Body_pos = ChVector<>(init_loc.x(), init_loc.y(), init_loc.z());
        if(i==0){
            Link_pos = Rover_Body_pos + wheel_rel_pos_lf;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+12];
        }
        if(i==1){
            Link_pos = Rover_Body_pos + wheel_rel_pos_rf;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+12];
        }
        if(i==2){
            Link_pos = Rover_Body_pos + wheel_rel_pos_lm;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+8];
        }
        if(i==3){
            Link_pos = Rover_Body_pos + wheel_rel_pos_rm;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+8];
        }
        if(i==4){
            Link_pos = Rover_Body_pos + wheel_rel_pos_lb;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+10];
        }
        if(i==5){
            Link_pos = Rover_Body_pos + wheel_rel_pos_rb;
            body_connected_to_wheel = sysMBS.Get_bodylist()[i+10];
        }

        if( i > 5 ){
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
            revo_link->Initialize(wheel, body_connected_to_wheel, ChCoordsys<>(Link_pos, z2y));
            sysMBS.AddLink(revo_link);
        }
        else{
            if(motor_type == 1){
                // sr-wheel Revolute constraint with a motor - Rotation Speed
                auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI*motor_F);  // speed w=3.145 rad/sec
                if(i==0){ 
                    link_motor_speed_lf = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_lf->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_lf);
                    link_motor_speed_lf->SetSpeedFunction(my_speed_function);
                }
                if(i==1){ 
                    link_motor_speed_rf = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_rf->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_rf);
                    link_motor_speed_rf->SetSpeedFunction(my_speed_function);
                }
                if(i==2){ 
                    link_motor_speed_lm = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_lm->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_lm);
                    link_motor_speed_lm->SetSpeedFunction(my_speed_function);
                }
                if(i==3){ 
                    link_motor_speed_rm = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_rm->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_rm);
                    link_motor_speed_rm->SetSpeedFunction(my_speed_function);
                }
                if(i==4){ 
                    link_motor_speed_lb = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_lb->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_lb);
                    link_motor_speed_lb->SetSpeedFunction(my_speed_function);
                }
                if(i==5){ 
                    link_motor_speed_rb = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
                    link_motor_speed_rb->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                    sysMBS.AddLink(link_motor_speed_rb);
                    link_motor_speed_rb->SetSpeedFunction(my_speed_function);
                }
            }
            else if(motor_type == 2){
                // sr-wheel Revolute constraint with a motor - torque
                auto link_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>(); 
                link_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
                link_motor->Initialize(body_connected_to_wheel, wheel, ChFrame<>(Link_pos, z2y));
                sysMBS.AddLink(link_motor);
                auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(link_motor->GetTorqueFunction());
                mfun->Set_yconst(motor_F);
            }
        }

    }

    // Create joint constraints for steering rod
    for (int i = 0; i < 4; i++) {
        // pick up bodies and create links
        auto s_rod = sysMBS.Get_bodylist()[i+12];
        auto body_connected_to_s_rod = sysMBS.Get_bodylist()[i+8];
        
        // define a rotation of -90 degrees around x (z2y)
        ChQuaternion<> z2y;
        z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));

        // pick up relative position of the link 
        ChVector<> Link_pos;
        ChVector<> Rover_Body_pos = ChVector<>(init_loc.x(), init_loc.y(), init_loc.z());
        if(i==0){
            Link_pos = Rover_Body_pos + sr_rel_pos_lf;
        }
        if(i==1){
            Link_pos = Rover_Body_pos + sr_rel_pos_rf;
        }
        if(i==2){ 
            Link_pos = Rover_Body_pos + sr_rel_pos_lb;
        }
        if(i==3){
            Link_pos = Rover_Body_pos + sr_rel_pos_rb;
        }
        // add Revolute constraint
        auto revo_link = chrono_types::make_shared<ChLinkLockLock>(); 
        revo_link->Initialize(s_rod, body_connected_to_s_rod, ChCoordsys<>(Link_pos, z2y));
        sysMBS.AddLink(revo_link);
    }

    // Create joint constraints for top rod with rover body
    {
        // pick up bodies and create links
        auto t_rod = sysMBS.Get_bodylist()[18];
        auto body_connected_to_t_rod = sysMBS.Get_bodylist()[1];

        // pick up relative position of the link 
        ChVector<> Link_pos;
        ChVector<> Rover_Body_pos = ChVector<>(init_loc.x(), init_loc.y(), init_loc.z());
        Link_pos = Rover_Body_pos + tr_rel_pos_t;

        // add Revolute constraint
        auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
        revo_link->Initialize(t_rod, body_connected_to_t_rod, ChCoordsys<>(Link_pos, ChQuaternion<>(1, 0, 0, 0)));
        sysMBS.AddLink(revo_link);


        // Revolute constraint with a motor - Rotation Speed
        // auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
        // link_motor->Initialize(body_connected_to_t_rod, t_rod, ChFrame<>(Link_pos, ChQuaternion<>(1, 0, 0, 0)));
        // sysMBS.AddLink(link_motor);
        // auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI*motor_F*0.1);  // speed w=3.145 rad/sec
        // link_motor->SetSpeedFunction(my_speed_function);

    }

    // Create joint constraints for top left/right rods with top rod
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            // pick up bodies and create links
            auto t_rod = sysMBS.Get_bodylist()[i+16];
            auto body_connected_to_t_rod = sysMBS.Get_bodylist()[0];
            
            // pick up relative position of the link 
            ChVector<> Link_pos;
            ChVector<> Rover_Body_pos = ChVector<>(init_loc.x(), init_loc.y(), init_loc.z());
            if(i==0 && j==0){
                Link_pos = Rover_Body_pos + tr_rel_pos_l;
                body_connected_to_t_rod = sysMBS.Get_bodylist()[8];
            }
            if(i==0 && j==1){
                Link_pos = Rover_Body_pos + tr_rel_pos_l + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_l.x(), 0, 0);
                body_connected_to_t_rod = sysMBS.Get_bodylist()[18];
            }
            if(i==1 && j==0){
                Link_pos = Rover_Body_pos + tr_rel_pos_r;
                body_connected_to_t_rod = sysMBS.Get_bodylist()[9];
            }
            if(i==1 && j==1){
                Link_pos = Rover_Body_pos + tr_rel_pos_r + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_r.x(), 0, 0);
                body_connected_to_t_rod = sysMBS.Get_bodylist()[18];
            }
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockSpherical>(); 
            revo_link->Initialize(t_rod, body_connected_to_t_rod, ChCoordsys<>(Link_pos, ChQuaternion<>(1, 0, 0, 0)));
            sysMBS.AddLink(revo_link);
        }
    }

    // Create joint constraints for connecting rod
    for (int i = 0; i < 4; i++) {
        // pick up bodies and create links
        auto c_rod = sysMBS.Get_bodylist()[i+8];
        auto body_connected_to_c_rod = sysMBS.Get_bodylist()[0];
        auto body_rover = sysMBS.Get_bodylist()[1];
        
        // define a rotation of -90 degrees around x (z2y)
        ChQuaternion<> z2y;
        z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));

        // pick up relative position of the link 
        ChVector<> Link_pos;
        ChVector<> Rover_Body_pos = ChVector<>(init_loc.x(), init_loc.y(), init_loc.z());
        if(i==0){
            Link_pos = Rover_Body_pos + cr_rel_pos_lf;
            body_connected_to_c_rod = sysMBS.Get_bodylist()[1];
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
            revo_link->Initialize(c_rod, body_connected_to_c_rod, ChCoordsys<>(Link_pos, z2y));
            sysMBS.AddLink(revo_link);
        }
        if(i==1){
            Link_pos = Rover_Body_pos + cr_rel_pos_rf;
            body_connected_to_c_rod = sysMBS.Get_bodylist()[1];
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
            revo_link->Initialize(c_rod, body_connected_to_c_rod, ChCoordsys<>(Link_pos, z2y));
            sysMBS.AddLink(revo_link);
        }
        if(i==2){ 
            Link_pos = Rover_Body_pos + cr_rel_pos_lb;
            body_connected_to_c_rod = sysMBS.Get_bodylist()[8];
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
            revo_link->Initialize(c_rod, body_connected_to_c_rod, ChCoordsys<>(Link_pos, z2y));
            sysMBS.AddLink(revo_link);
        }
        if(i==3){
            Link_pos = Rover_Body_pos + cr_rel_pos_rb;
            body_connected_to_c_rod = sysMBS.Get_bodylist()[9];
            // add Revolute constraint
            auto revo_link = chrono_types::make_shared<ChLinkLockRevolute>(); 
            revo_link->Initialize(c_rod, body_connected_to_c_rod, ChCoordsys<>(Link_pos, z2y));
            sysMBS.AddLink(revo_link);
        }
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
