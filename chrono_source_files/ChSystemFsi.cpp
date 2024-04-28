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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiInterface.h"
#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFluid.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChSystemFsi::ChSystemFsi(ChSystem* sysMBS)
    : m_sysMBS(sysMBS),
      m_verbose(true),
      m_is_initialized(false),
      m_integrate_SPH(true),
      m_time(0),
      m_write_mode(OutpuMode::NONE) {
    m_paramsH = chrono_types::make_shared<SimParams>();
    m_sysFSI = chrono_types::make_unique<ChSystemFsi_impl>(m_paramsH);
    InitParams();
    m_num_objectsH = m_sysFSI->numObjects;

    m_num_cable_elements = 0;
    m_num_shell_elements = 0;

    m_fsi_interface = chrono_types::make_unique<ChFsiInterface>(*m_sysFSI, m_paramsH);
}

ChSystemFsi::~ChSystemFsi() {}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::InitParams() {
    //// RADU TODO
    //// Provide default values for *all* parameters!

    m_paramsH->output_length = 1;

    // Fluid properties
    m_paramsH->rho0 = Real(1000.0);
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->rho_solid = m_paramsH->rho0;
    m_paramsH->mu0 = Real(0.001);
    m_paramsH->bodyForce3 = mR3(0, 0, 0);
    m_paramsH->gravity = mR3(0, 0, 0);
    m_paramsH->kappa = Real(0.0);
    m_paramsH->L_Characteristic = Real(1.0);

    // SPH parameters
    m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
    m_paramsH->HSML = Real(0.01);
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INITSPACE = m_paramsH->HSML;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->MULT_INITSPACE_Shells = Real(1.0);
    m_paramsH->v_Max = Real(1.0);
    m_paramsH->EPS_XSPH = Real(0.5);
    m_paramsH->beta_shifting = Real(1.0);
    m_paramsH->densityReinit = 2147483647;
    m_paramsH->Conservative_Form = true;
    m_paramsH->gradient_type = 0;
    m_paramsH->laplacian_type = 0;
    m_paramsH->USE_Consistent_L = false;
    m_paramsH->USE_Consistent_G = false;

    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->NUM_BOUNDARY_LAYERS = 3;

    // Time stepping
    m_paramsH->Adaptive_time_stepping = false;
    m_paramsH->Co_number = Real(0.1);
    m_paramsH->Beta = Real(0.0);
    m_paramsH->dT = Real(0.0001);
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = m_paramsH->dT;
    m_paramsH->dT_Max = Real(1.0);

    // Pressure equation
    m_paramsH->PPE_Solution_type = PPESolutionType::MATRIX_FREE;
    m_paramsH->Alpha = m_paramsH->HSML;
    m_paramsH->PPE_relaxation = Real(1.0);
    m_paramsH->LinearSolver = SolverType::BICGSTAB;
    m_paramsH->LinearSolver_Abs_Tol = Real(0.0);
    m_paramsH->LinearSolver_Rel_Tol = Real(0.0);
    m_paramsH->LinearSolver_Max_Iter = 1000;
    m_paramsH->Verbose_monitoring = false;
    m_paramsH->Pressure_Constraint = false;
    m_paramsH->BASEPRES = Real(0.0);
    m_paramsH->ClampPressure = false;

    m_paramsH->bceType = BceVersion::ADAMI;
    m_paramsH->bceTypeWall = BceVersion::ADAMI;

    // Elastic SPH
    m_paramsH->C_Wi = Real(0.8);

    //
    m_paramsH->bodyActiveDomain = mR3(1e10, 1e10, 1e10);
    m_paramsH->settlingTime = Real(1e10);

    //
    m_paramsH->Max_Pressure = Real(1e20);

    //// RADU TODO
    //// material model

    // Elastic SPH
    ElasticMaterialProperties mat_props;
    SetElasticSPH(mat_props);
    m_paramsH->elastic_SPH = false;  // default: fluid dynamics

    m_paramsH->Cs = 10 * m_paramsH->v_Max;

    m_paramsH->use_default_limits = true;
    m_paramsH->use_init_pressure = false;
}

//--------------------------------------------------------------------------------------------------------------------------------

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

void ChSystemFsi::ReadParametersFromFile(const std::string& json_file) {
    if (m_verbose)
        cout << "Reading parameters from: " << json_file << endl;

    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        cerr << "Invalid JSON file!" << endl;
        return;
    }

    char readBuffer[32768];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cerr << "Invalid JSON file!!" << endl;
        return;
    }

    if (doc.HasMember("Data Output Length"))
        m_paramsH->output_length = doc["Data Output Length"].GetInt();

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            m_paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Solid Density"))
            m_paramsH->rho_solid = doc["Physical Properties of Fluid"]["Solid Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            m_paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            m_paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            m_paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);

        if (doc["Physical Properties of Fluid"].HasMember("Surface Tension Kappa"))
            m_paramsH->kappa = doc["Physical Properties of Fluid"]["Surface Tension Kappa"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            m_paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            if (m_verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::I2SPH;
            else if (SPH == "IISPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::IISPH;
            else if (SPH == "WCSPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to WCSPH " << endl;
                m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Kernel h"))
            m_paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            m_paramsH->INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing Solid"))
            m_paramsH->MULT_INITSPACE_Shells =
                doc["SPH Parameters"]["Initial Spacing Solid"].GetDouble() / m_paramsH->HSML;

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            m_paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();
        else
            m_paramsH->epsMinMarkersDis = 0.01;

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            m_paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            m_paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Viscous damping"))
            m_paramsH->Vis_Dam = doc["SPH Parameters"]["Viscous damping"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
            m_paramsH->beta_shifting = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Density Reinitialization"))
            m_paramsH->densityReinit = doc["SPH Parameters"]["Density Reinitialization"].GetInt();

        if (doc["SPH Parameters"].HasMember("Conservative Discretization"))
            m_paramsH->Conservative_Form = doc["SPH Parameters"]["Conservative Discretization"].GetBool();

        if (doc["SPH Parameters"].HasMember("Gradient Discretization Type"))
            m_paramsH->gradient_type = doc["SPH Parameters"]["Gradient Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Laplacian Discretization Type"))
            m_paramsH->laplacian_type = doc["SPH Parameters"]["Laplacian Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Laplacian"))
            m_paramsH->USE_Consistent_L = doc["SPH Parameters"]["Consistent Discretization for Laplacian"].GetBool();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Gradient"))
            m_paramsH->USE_Consistent_G = doc["SPH Parameters"]["Consistent Discretization for Gradient"].GetBool();
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Adaptive Time stepping"))
            m_paramsH->Adaptive_time_stepping = doc["Time Stepping"]["Adaptive Time stepping"].GetBool();

        if (doc["Time Stepping"].HasMember("CFL number"))
            m_paramsH->Co_number = doc["Time Stepping"]["CFL number"].GetDouble();

        if (doc["Time Stepping"].HasMember("Beta"))
            m_paramsH->Beta = doc["Time Stepping"]["Beta"].GetDouble();

        if (doc["Time Stepping"].HasMember("Fluid time step"))
            m_paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();

        if (doc["Time Stepping"].HasMember("Solid time step"))
            m_paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            m_paramsH->dT_Flex = m_paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            m_paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            m_paramsH->PPE_Solution_type = PPESolutionType::FORM_SPARSE_MATRIX;
            std::string solver = doc["Pressure Equation"]["Linear solver"].GetString();
            if (solver == "Jacobi") {
                m_paramsH->USE_LinearSolver = false;
            } else {
                m_paramsH->USE_LinearSolver = true;
                if (solver == "BICGSTAB")
                    m_paramsH->LinearSolver = SolverType::BICGSTAB;
                if (solver == "GMRES")
                    m_paramsH->LinearSolver = SolverType::GMRES;
            }
        }

        if (doc["Pressure Equation"].HasMember("Poisson source term")) {
            std::string source = doc["Pressure Equation"]["Poisson source term"].GetString();
            if (source == "Density-Based")
                m_paramsH->DensityBaseProjetion = true;
            else
                m_paramsH->DensityBaseProjetion = false;
        }

        if (doc["Pressure Equation"].HasMember("Projection method")) {
            std::string source = doc["Pressure Equation"]["Projection method"].GetString();
            if (source == "Incremental")
                m_paramsH->USE_NonIncrementalProjection = false;
            else
                m_paramsH->USE_NonIncrementalProjection = true;
        }

        if (doc["Pressure Equation"].HasMember("Alpha Source Term"))
            m_paramsH->Alpha = doc["Pressure Equation"]["Alpha Source Term"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Under-relaxation"))
            m_paramsH->PPE_relaxation = doc["Pressure Equation"]["Under-relaxation"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Absolute residual"))
            m_paramsH->LinearSolver_Abs_Tol = doc["Pressure Equation"]["Absolute residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Relative residual"))
            m_paramsH->LinearSolver_Rel_Tol = doc["Pressure Equation"]["Relative residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Maximum Iterations"))
            m_paramsH->LinearSolver_Max_Iter = doc["Pressure Equation"]["Maximum Iterations"].GetInt();

        if (doc["Pressure Equation"].HasMember("Verbose monitoring"))
            m_paramsH->Verbose_monitoring = doc["Pressure Equation"]["Verbose monitoring"].GetBool();

        if (doc["Pressure Equation"].HasMember("Constraint Pressure")) {
            m_paramsH->Pressure_Constraint = doc["Pressure Equation"]["Constraint Pressure"].GetBool();
            if (doc["Pressure Equation"].HasMember("Average Pressure"))
                m_paramsH->BASEPRES = doc["Pressure Equation"]["Average Pressure"].GetDouble();
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            m_paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();

        if (doc["Pressure Equation"].HasMember("Boundary Conditions")) {
            std::string BC = doc["Pressure Equation"]["Boundary Conditions"].GetString();
            if (BC == "Generalized Wall BC")
                m_paramsH->bceType = BceVersion::ADAMI;
            else
                m_paramsH->bceType = BceVersion::ORIGINAL;
        }
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        m_paramsH->elastic_SPH = true;

        if (doc["Elastic SPH"].HasMember("Poisson ratio"))
            m_paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Young modulus"))
            m_paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial stress"))
            m_paramsH->Ar_stress = doc["Elastic SPH"]["Artificial stress"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha"))
            m_paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity beta"))
            m_paramsH->Ar_vis_beta = doc["Elastic SPH"]["Artificial viscosity beta"].GetDouble();

        if (doc["Elastic SPH"].HasMember("I0"))
            m_paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_s"))
            m_paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_2"))
            m_paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();

        if (doc["Elastic SPH"].HasMember("particle diameter"))
            m_paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();

        if (doc["Elastic SPH"].HasMember("frictional angle"))
            m_paramsH->Fri_angle = doc["Elastic SPH"]["frictional angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("dilate angle"))
            m_paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("cohesion coefficient"))
            m_paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();

        if (doc["Elastic SPH"].HasMember("kernel threshold"))
            m_paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble();
    }

    // Geometry Information
    if (doc.HasMember("Geometry Inf")) {
        if (doc["Geometry Inf"].HasMember("BoxDimensionX"))
            m_paramsH->boxDimX = doc["Geometry Inf"]["BoxDimensionX"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionY"))
            m_paramsH->boxDimY = doc["Geometry Inf"]["BoxDimensionY"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionZ"))
            m_paramsH->boxDimZ = doc["Geometry Inf"]["BoxDimensionZ"].GetDouble();
    }

    if (doc.HasMember("Body Active Domain"))
        m_paramsH->bodyActiveDomain = LoadVectorJSON(doc["Body Active Domain"]);

    if (doc.HasMember("Settling Time"))
        m_paramsH->settlingTime = doc["Settling Time"].GetDouble();

    //===============================================================
    // Material Models
    //===============================================================
    if (doc.HasMember("Material Model")) {
        m_paramsH->non_newtonian = doc["Material Model"]["Non-Newtonian"].GetBool();
        //===============================================================
        // For a simple non-newtonian flow
        //==============================================================
        if (m_paramsH->non_newtonian) {
            m_paramsH->mu_max = doc["Material Model"]["max Viscosity"].GetDouble();

            if (m_paramsH->non_newtonian) {
                if (doc["Material Model"].HasMember("HerschelBulkley")) {
                    m_paramsH->HB_k = doc["Material Model"]["HerschelBulkley"]["k"].GetDouble();
                    m_paramsH->HB_n = doc["Material Model"]["HerschelBulkley"]["n"].GetInt();
                    m_paramsH->HB_tau0 = doc["Material Model"]["HerschelBulkley"]["tau_0"].GetDouble();
                    if (doc["Material Model"]["HerschelBulkley"].HasMember("sr0"))
                        m_paramsH->HB_sr0 = doc["Material Model"]["HerschelBulkley"]["sr0"].GetDouble();
                    else
                        m_paramsH->HB_sr0 = 0.0;
                } else {
                    if (m_verbose)
                        cout << "Constants of HerschelBulkley not found. Using default Newtonian values." << endl;
                    m_paramsH->HB_k = m_paramsH->mu0;
                    m_paramsH->HB_n = 1;
                    m_paramsH->HB_tau0 = 0;
                    m_paramsH->HB_sr0 = 0.0;
                }
            }
        }
    } else {
        m_paramsH->non_newtonian = false;
    }

    // Calculate dependent parameters
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;

    if (m_paramsH->elastic_SPH) {
        m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
        m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
        m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

        Real sfri = std::sin(m_paramsH->Fri_angle);
        Real cfri = std::cos(m_paramsH->Fri_angle);
        Real sdil = std::sin(m_paramsH->Dil_angle);
        m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
        m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
        m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::SetVerbose(bool verb) {
    m_verbose = verb;
    m_fsi_interface->m_verbose = verb;
}

void ChSystemFsi::SetSPHLinearSolver(SolverType lin_solver) {
    m_paramsH->LinearSolver = lin_solver;
}

void ChSystemFsi::SetSPHMethod(FluidDynamics SPH_method, SolverType lin_solver) {
    m_paramsH->fluid_dynamic_type = SPH_method;
    m_paramsH->LinearSolver = lin_solver;
}

void ChSystemFsi::SetContainerDim(const ChVector<>& boxDim) {
    m_paramsH->boxDimX = boxDim.x();
    m_paramsH->boxDimY = boxDim.y();
    m_paramsH->boxDimZ = boxDim.z();
}

void ChSystemFsi::SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax) {
    m_paramsH->cMin = utils::ToReal3(cMin);
    m_paramsH->cMax = utils::ToReal3(cMax);
    m_paramsH->use_default_limits = false;
}

void ChSystemFsi::SetActiveDomain(const ChVector<>& boxDim) {
    m_paramsH->bodyActiveDomain = utils::ToReal3(boxDim);
}

void ChSystemFsi::SetNumBoundaryLayers(int num_layers) {
    m_paramsH->NUM_BOUNDARY_LAYERS = num_layers;
}

void ChSystemFsi::SetInitPressure(const double height) {
    m_paramsH->pressure_height = height;
    m_paramsH->use_init_pressure = true;
}

void ChSystemFsi::Set_G_acc(const ChVector<>& gravity) {
    m_paramsH->gravity.x = gravity.x();
    m_paramsH->gravity.y = gravity.y();
    m_paramsH->gravity.z = gravity.z();
}

void ChSystemFsi::SetBodyForce(const ChVector<>& force) {
    m_paramsH->bodyForce3.x = force.x();
    m_paramsH->bodyForce3.y = force.y();
    m_paramsH->bodyForce3.z = force.z();
}

void ChSystemFsi::SetInitialSpacing(double spacing) {
    m_paramsH->INITSPACE = (Real)spacing;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChSystemFsi::SetKernelLength(double length) {
    m_paramsH->HSML = (Real)length;
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
}

void ChSystemFsi::SetStepSize(double dT, double dT_Flex) {
    m_paramsH->dT = dT;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = (dT_Flex == 0) ? m_paramsH->dT : dT_Flex;
}

void ChSystemFsi::SetMaxStepSize(double dT_max) {
    m_paramsH->dT_Max = Real(dT_max);
}

void ChSystemFsi::SetAdaptiveTimeStepping(bool adaptive) {
    m_paramsH->Adaptive_time_stepping = adaptive;
}

void ChSystemFsi::SetSPHintegration(bool runSPH) {
    m_integrate_SPH = runSPH;
}

void ChSystemFsi::SetDensity(double rho0) {
    m_paramsH->rho0 = rho0;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChSystemFsi::SetDiscreType(bool useGmatrix, bool useLmatrix) {
    m_paramsH->USE_Consistent_G = useGmatrix;
    m_paramsH->USE_Consistent_L = useLmatrix;
}

void ChSystemFsi::SetOutputLength(int OutputLength) {
    m_paramsH->output_length = OutputLength;
}

void ChSystemFsi::SetWallBC(BceVersion wallBC) {
    m_paramsH->bceTypeWall = wallBC;
}

void ChSystemFsi::SetRigidBodyBC(BceVersion rigidBodyBC) {
    m_paramsH->bceType = rigidBodyBC;
}

void ChSystemFsi::SetCohesionForce(double Fc) {
    m_paramsH->Coh_coeff = Fc;
}

ChSystemFsi::ElasticMaterialProperties::ElasticMaterialProperties()
    : Young_modulus(1e6),
      Poisson_ratio(0.3),
      stress(0),
      viscosity_alpha(0.5),
      viscosity_beta(0),
      mu_I0(0.03),
      mu_fric_s(0.7),
      mu_fric_2(0.7),
      average_diam(0.005),
      friction_angle(CH_C_PI / 10),
      dilation_angle(CH_C_PI / 10),
      cohesion_coeff(0),
      kernel_threshold(0.8) {}

void ChSystemFsi::SetElasticSPH(const ElasticMaterialProperties mat_props) {
    m_paramsH->elastic_SPH = true;

    m_paramsH->E_young = Real(mat_props.Young_modulus);
    m_paramsH->Nu_poisson = Real(mat_props.Poisson_ratio);
    m_paramsH->Ar_stress = Real(mat_props.stress);
    m_paramsH->Ar_vis_alpha = Real(mat_props.viscosity_alpha);
    m_paramsH->Ar_vis_beta = Real(mat_props.viscosity_beta);
    m_paramsH->mu_I0 = Real(mat_props.mu_I0);
    m_paramsH->mu_fric_s = Real(mat_props.mu_fric_s);
    m_paramsH->mu_fric_2 = Real(mat_props.mu_fric_2);
    m_paramsH->ave_diam = Real(mat_props.average_diam);
    m_paramsH->Fri_angle = Real(mat_props.friction_angle);
    m_paramsH->Dil_angle = Real(mat_props.dilation_angle);
    m_paramsH->Coh_coeff = Real(mat_props.cohesion_coeff);
    m_paramsH->C_Wi = Real(mat_props.kernel_threshold);

    m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
    m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
    m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
    m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

    Real sfri = std::sin(m_paramsH->Fri_angle);
    Real cfri = std::cos(m_paramsH->Fri_angle);
    Real sdil = std::sin(m_paramsH->Dil_angle);
    m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
    m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
    m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddFsiBody(std::shared_ptr<ChBody> body) {
    m_fsi_interface->m_fsi_bodies.push_back(body);
}

void ChSystemFsi::AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh,
                             const std::vector<std::vector<int>>& beam_elements,
                             const std::vector<std::vector<int>>& shell_elements) {
    m_fsi_interface->m_fsi_mesh = mesh;

    m_fea_cable_nodes = beam_elements;
    m_fea_shell_nodes = shell_elements;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Initialize() {
    // Calculate the initial neighour particle number
    m_paramsH->markerMass = utils::massCalculator(m_paramsH->HSML, m_paramsH->INITSPACE, m_paramsH->rho0);
    m_paramsH->num_neighbors = utils::IniNeiNum(m_paramsH->HSML, m_paramsH->INITSPACE);

    if (m_paramsH->use_default_limits) {
        m_paramsH->cMin =
            mR3(-2 * m_paramsH->boxDimX, -2 * m_paramsH->boxDimY, -2 * m_paramsH->boxDimZ) - 10 * mR3(m_paramsH->HSML);
        m_paramsH->cMax =
            mR3(+2 * m_paramsH->boxDimX, +2 * m_paramsH->boxDimY, +2 * m_paramsH->boxDimZ) + 10 * mR3(m_paramsH->HSML);
    }

    if (m_paramsH->use_init_pressure) {
        size_t numParticles = m_sysFSI->sphMarkersH->rhoPresMuH.size();
        for (int i = 0; i < numParticles; i++) {
            double z = m_sysFSI->sphMarkersH->posRadH[i].z;
            double p = m_paramsH->rho0 * m_paramsH->gravity.z * (z - m_paramsH->pressure_height);
            m_sysFSI->sphMarkersH->rhoPresMuH[i].y = p;
            if (m_paramsH->elastic_SPH) {
                m_sysFSI->sphMarkersH->tauXxYyZzH[i].x = -p;
                m_sysFSI->sphMarkersH->tauXxYyZzH[i].y = -p;
                m_sysFSI->sphMarkersH->tauXxYyZzH[i].z = -p;
            }
        }
    }

    // Set up subdomains for faster neighbor particle search
    m_paramsH->NUM_BOUNDARY_LAYERS = 3;
    m_paramsH->Apply_BC_U = false;  // You should go to custom_math.h all the way to end of file and set your function
    int3 side0 = mI3((int)floor((m_paramsH->cMax.x - m_paramsH->cMin.x) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.y - m_paramsH->cMin.y) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.z - m_paramsH->cMin.z) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)));
    Real3 binSize3 =
        mR3((m_paramsH->cMax.x - m_paramsH->cMin.x) / side0.x, (m_paramsH->cMax.y - m_paramsH->cMin.y) / side0.y,
            (m_paramsH->cMax.z - m_paramsH->cMin.z) / side0.z);
    m_paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    m_paramsH->binSize0 = binSize3.x;
    m_paramsH->boxDims = m_paramsH->cMax - m_paramsH->cMin;
    m_paramsH->straightChannelBoundaryMin = m_paramsH->cMin;  // mR3(0, 0, 0);  // 3D channel
    m_paramsH->straightChannelBoundaryMax = m_paramsH->cMax;  // SmR3(3, 2, 3) * m_paramsH->sizeScale;
    m_paramsH->deltaPress = mR3(0);
    int3 SIDE = mI3(int((m_paramsH->cMax.x - m_paramsH->cMin.x) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.y - m_paramsH->cMin.y) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.z - m_paramsH->cMin.z) / m_paramsH->binSize0 + .1));
    Real mBinSize = m_paramsH->binSize0;
    m_paramsH->gridSize = SIDE;
    m_paramsH->worldOrigin = m_paramsH->cMin;
    m_paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);

    // Print information
    if (m_verbose) {
        cout << "Simulation parameters" << endl;

        cout << "  num_neighbors: " << m_paramsH->num_neighbors << endl;
        cout << "  rho0: " << m_paramsH->rho0 << endl;
        cout << "  invrho0: " << m_paramsH->invrho0 << endl;
        cout << "  mu0: " << m_paramsH->mu0 << endl;
        cout << "  bodyForce3: " << m_paramsH->bodyForce3.x << " " << m_paramsH->bodyForce3.y << " "
             << m_paramsH->bodyForce3.z << endl;
        cout << "  gravity: " << m_paramsH->gravity.x << " " << m_paramsH->gravity.y << " " << m_paramsH->gravity.z
             << endl;

        cout << "  HSML: " << m_paramsH->HSML << endl;
        cout << "  INITSPACE: " << m_paramsH->INITSPACE << endl;
        cout << "  INV_INIT: " << m_paramsH->INV_INIT << endl;
        cout << "  MULT_INITSPACE: " << m_paramsH->MULT_INITSPACE << endl;
        cout << "  NUM_BOUNDARY_LAYERS: " << m_paramsH->NUM_BOUNDARY_LAYERS << endl;
        cout << "  epsMinMarkersDis: " << m_paramsH->epsMinMarkersDis << endl;
        cout << "  markerMass: " << m_paramsH->markerMass << endl;
        cout << "  volume0: " << m_paramsH->volume0 << endl;
        cout << "  gradient_type: " << m_paramsH->gradient_type << endl;

        cout << "  v_Max: " << m_paramsH->v_Max << endl;
        cout << "  Cs: " << m_paramsH->Cs << endl;
        cout << "  EPS_XSPH: " << m_paramsH->EPS_XSPH << endl;
        cout << "  beta_shifting: " << m_paramsH->beta_shifting << endl;
        cout << "  densityReinit: " << m_paramsH->densityReinit << endl;

        cout << "  Adaptive_time_stepping: " << m_paramsH->Adaptive_time_stepping << endl;
        cout << "  Co_number: " << m_paramsH->Co_number << endl;
        cout << "  dT: " << m_paramsH->dT << endl;
        cout << "  INV_dT: " << m_paramsH->INV_dT << endl;
        cout << "  dT_Max: " << m_paramsH->dT_Max << endl;
        cout << "  dT_Flex: " << m_paramsH->dT_Flex << endl;

        cout << "  non_newtonian: " << m_paramsH->non_newtonian << endl;
        cout << "  mu_of_I : " << (int)m_paramsH->mu_of_I << endl;
        cout << "  rheology_model: " << (int)m_paramsH->rheology_model << endl;
        cout << "  ave_diam: " << m_paramsH->ave_diam << endl;
        cout << "  mu_max: " << m_paramsH->mu_max << endl;
        cout << "  mu_fric_s: " << m_paramsH->mu_fric_s << endl;
        cout << "  mu_fric_2: " << m_paramsH->mu_fric_2 << endl;
        cout << "  mu_I0: " << m_paramsH->mu_I0 << endl;
        cout << "  mu_I_b: " << m_paramsH->mu_I_b << endl;
        cout << "  HB_k: " << m_paramsH->HB_k << endl;
        cout << "  HB_n: " << m_paramsH->HB_n << endl;
        cout << "  HB_tau0: " << m_paramsH->HB_tau0 << endl;

        cout << "  E_young: " << m_paramsH->E_young << endl;
        cout << "  G_shear: " << m_paramsH->G_shear << endl;
        cout << "  INV_G_shear: " << m_paramsH->INV_G_shear << endl;
        cout << "  K_bulk: " << m_paramsH->K_bulk << endl;
        cout << "  C_Wi: " << m_paramsH->C_Wi << endl;

        cout << "  bceType: " << (int)m_paramsH->bceType << endl;
        cout << "  USE_NonIncrementalProjection : " << m_paramsH->USE_NonIncrementalProjection << endl;
        cout << "  PPE_relaxation: " << m_paramsH->PPE_relaxation << endl;
        cout << "  Conservative_Form: " << m_paramsH->Conservative_Form << endl;
        cout << "  Pressure_Constraint: " << m_paramsH->Pressure_Constraint << endl;

        cout << "  binSize0: " << m_paramsH->binSize0 << endl;
        cout << "  boxDims: " << m_paramsH->boxDims.x << " " << m_paramsH->boxDims.y << " " << m_paramsH->boxDims.z
             << endl;
        cout << "  gridSize: " << m_paramsH->gridSize.x << " " << m_paramsH->gridSize.y << " " << m_paramsH->gridSize.z
             << endl;
        cout << "  cMin: " << m_paramsH->cMin.x << " " << m_paramsH->cMin.y << " " << m_paramsH->cMin.z << endl;
        cout << "  cMax: " << m_paramsH->cMax.x << " " << m_paramsH->cMax.y << " " << m_paramsH->cMax.z << endl;
    }

    // Resize worker data
    m_fsi_interface->ResizeChronoCablesData(m_fea_cable_nodes);
    m_fsi_interface->ResizeChronoShellsData(m_fea_shell_nodes);

    // This also sets the referenceArray and counts numbers of various objects
    size_t n_flexnodes = m_fsi_interface->m_fsi_mesh ? (size_t)m_fsi_interface->m_fsi_mesh->GetNnodes() : 0;
    m_sysFSI->ResizeData(m_fsi_interface->m_fsi_bodies.size(), m_num_cable_elements, m_num_shell_elements, n_flexnodes);

    if (m_verbose) {
        cout << "Counters" << endl;
        cout << "  numRigidBodies: " << m_sysFSI->numObjects->numRigidBodies << endl;
        cout << "  numFlexNodes: " << m_sysFSI->numObjects->numFlexNodes << endl;
        cout << "  numFlexBodies1D: " << m_sysFSI->numObjects->numFlexBodies1D << endl;
        cout << "  numFlexBodies2D: " << m_sysFSI->numObjects->numFlexBodies2D << endl;
        cout << "  numGhostMarkers: " << m_sysFSI->numObjects->numGhostMarkers << endl;
        cout << "  numHelperMarkers: " << m_sysFSI->numObjects->numHelperMarkers << endl;
        cout << "  numFluidMarkers: " << m_sysFSI->numObjects->numFluidMarkers << endl;
        cout << "  numBoundaryMarkers: " << m_sysFSI->numObjects->numBoundaryMarkers << endl;
        cout << "  numRigidMarkers: " << m_sysFSI->numObjects->numRigidMarkers << endl;
        cout << "  numFlexMarkers: " << m_sysFSI->numObjects->numFlexMarkers << endl;
        cout << "  numAllMarkers: " << m_sysFSI->numObjects->numAllMarkers << endl;
        cout << "  startRigidMarkers: " << m_sysFSI->numObjects->startRigidMarkers << endl;
        cout << "  startFlexMarkers: " << m_sysFSI->numObjects->startFlexMarkers << endl;

        cout << "Reference array (size: " << m_sysFSI->fsiGeneralData->referenceArray.size() << ")" << endl;
        for (size_t i = 0; i < m_sysFSI->fsiGeneralData->referenceArray.size(); i++) {
            const int4& num = m_sysFSI->fsiGeneralData->referenceArray[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << "Reference array FEA (size: " << m_sysFSI->fsiGeneralData->referenceArray_FEA.size() << ")" << endl;
        for (size_t i = 0; i < m_sysFSI->fsiGeneralData->referenceArray_FEA.size(); i++) {
            const int4& num = m_sysFSI->fsiGeneralData->referenceArray_FEA[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
    }

    m_fsi_interface->Copy_FsiBodies_ChSystem_to_FsiSystem(m_sysFSI->fsiBodiesD1);
    m_fsi_interface->Copy_FsiNodes_ChSystem_to_FsiSystem(m_sysFSI->fsiMeshD);

    // Construct midpoint rigid data
    m_sysFSI->fsiBodiesD2 = m_sysFSI->fsiBodiesD1;

    // Create BCE and SPH worker objects
    m_bce_manager = chrono_types::make_shared<ChBce>(m_sysFSI->sortedSphMarkersD, m_sysFSI->markersProximityD,
                                                     m_sysFSI->fsiGeneralData, m_paramsH, m_num_objectsH, m_verbose);

    switch (m_paramsH->fluid_dynamic_type) {
        case FluidDynamics::IISPH:
            fluidIntegrator = TimeIntegrator::IISPH;
            break;
        case FluidDynamics::WCSPH:
            fluidIntegrator = TimeIntegrator::EXPLICITSPH;
            break;
        default:
            fluidIntegrator = TimeIntegrator::I2SPH;
            break;
    }
    m_fluid_dynamics = chrono_types::make_unique<ChFluidDynamics>(m_bce_manager, *m_sysFSI, m_paramsH, m_num_objectsH,
                                                                  fluidIntegrator, m_verbose);
    m_fluid_dynamics->GetForceSystem()->SetLinearSolver(m_paramsH->LinearSolver);

    // Initialize worker objects
    m_bce_manager->Initialize(m_sysFSI->sphMarkersD1, m_sysFSI->fsiBodiesD1, m_sysFSI->fsiMeshD, m_fsi_bodies_bce_num,
                              m_fsi_shells_bce_num, m_fsi_cables_bce_num);
    m_fluid_dynamics->Initialize();

    // Mark system as initialized
    m_is_initialized = true;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CopyDeviceDataToHalfStep() {
    thrust::copy(m_sysFSI->sphMarkersD2->posRadD.begin(), m_sysFSI->sphMarkersD2->posRadD.end(),
                 m_sysFSI->sphMarkersD1->posRadD.begin());
    thrust::copy(m_sysFSI->sphMarkersD2->velMasD.begin(), m_sysFSI->sphMarkersD2->velMasD.end(),
                 m_sysFSI->sphMarkersD1->velMasD.begin());
    thrust::copy(m_sysFSI->sphMarkersD2->rhoPresMuD.begin(), m_sysFSI->sphMarkersD2->rhoPresMuD.end(),
                 m_sysFSI->sphMarkersD1->rhoPresMuD.begin());
    if (m_paramsH->elastic_SPH) {
        thrust::copy(m_sysFSI->sphMarkersD2->tauXxYyZzD.begin(), m_sysFSI->sphMarkersD2->tauXxYyZzD.end(),
                     m_sysFSI->sphMarkersD1->tauXxYyZzD.begin());
        thrust::copy(m_sysFSI->sphMarkersD2->tauXyXzYzD.begin(), m_sysFSI->sphMarkersD2->tauXyXzYzD.end(),
                     m_sysFSI->sphMarkersD1->tauXyXzYzD.begin());
    }
}


void ChSystemFsi::DoStepDynamics_FSI() {
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    if (m_fluid_dynamics->GetIntegratorType() == TimeIntegrator::EXPLICITSPH) {
        // The following is used to execute the Explicit WCSPH
        CopyDeviceDataToHalfStep();
        thrust::copy(m_sysFSI->fsiGeneralData->derivVelRhoD.begin(), m_sysFSI->fsiGeneralData->derivVelRhoD.end(),
                     m_sysFSI->fsiGeneralData->derivVelRhoD_old.begin());
        ChUtilsDevice::FillVector(m_sysFSI->fsiGeneralData->derivVelRhoD, mR4(0));

        if (m_integrate_SPH) {
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkersD2, m_sysFSI->sphMarkersD1, m_sysFSI->fsiBodiesD2,
                                           m_sysFSI->fsiMeshD, 0.5 * m_paramsH->dT, m_time);
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkersD1, m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2,
                                           m_sysFSI->fsiMeshD, 1.0 * m_paramsH->dT, m_time);
        }

        m_bce_manager->Rigid_Forces_Torques(m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2);
        m_bce_manager->Flex_Forces(m_sysFSI->sphMarkersD2, m_sysFSI->fsiMeshD);

        // Advance dynamics of the associated MBS system (if provided)
        if (m_sysMBS) {
            m_fsi_interface->Add_Rigid_ForceTorques_To_ChSystem();
            m_fsi_interface->Add_Flex_Forces_To_ChSystem();

            if (m_paramsH->dT_Flex == 0)
                m_paramsH->dT_Flex = m_paramsH->dT;
            int sync = int(m_paramsH->dT / m_paramsH->dT_Flex);
            if (sync < 1)
                sync = 1;
            for (int t = 0; t < sync; t++) {
                m_sysMBS->DoStepDynamics(m_paramsH->dT / sync);
            }
        }

        m_fsi_interface->Copy_FsiBodies_ChSystem_to_FsiSystem(m_sysFSI->fsiBodiesD2);
        m_bce_manager->UpdateRigidMarkersPositionVelocity(m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2);

        m_fsi_interface->Copy_FsiNodes_ChSystem_to_FsiSystem(m_sysFSI->fsiMeshD);
        m_bce_manager->UpdateFlexMarkersPositionVelocity(m_sysFSI->sphMarkersD2, m_sysFSI->fsiMeshD);
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        if (m_integrate_SPH) {
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkersD2, m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2,
                                           m_sysFSI->fsiMeshD, 0.0, m_time);
        }

        m_bce_manager->Rigid_Forces_Torques(m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2);
        m_bce_manager->Flex_Forces(m_sysFSI->sphMarkersD2, m_sysFSI->fsiMeshD);

        // Advance dynamics of the associated MBS system (if provided)
        if (m_sysMBS) {
            m_fsi_interface->Add_Rigid_ForceTorques_To_ChSystem();
            m_fsi_interface->Add_Flex_Forces_To_ChSystem();

            if (m_paramsH->dT_Flex == 0)
                m_paramsH->dT_Flex = m_paramsH->dT;
            int sync = int(m_paramsH->dT / m_paramsH->dT_Flex);
            if (sync < 1)
                sync = 1;
            if (m_verbose)
                cout << sync << " * Chrono StepDynamics with dt = " << m_paramsH->dT / sync << endl;
            for (int t = 0; t < sync; t++) {
                m_sysMBS->DoStepDynamics(m_paramsH->dT / sync);
            }
        }

        m_fsi_interface->Copy_FsiBodies_ChSystem_to_FsiSystem(m_sysFSI->fsiBodiesD2);
        m_bce_manager->UpdateRigidMarkersPositionVelocity(m_sysFSI->sphMarkersD2, m_sysFSI->fsiBodiesD2);

        m_fsi_interface->Copy_FsiNodes_ChSystem_to_FsiSystem(m_sysFSI->fsiMeshD);
        m_bce_manager->UpdateFlexMarkersPositionVelocity(m_sysFSI->sphMarkersD2, m_sysFSI->fsiMeshD);
    }

    m_time += 1 * m_paramsH->dT;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::WriteParticleFile(const std::string& outfilename) const {
    if (m_write_mode == OutpuMode::CSV) {
        utils::WriteCsvParticlesToFile(m_sysFSI->sphMarkersD2->posRadD, m_sysFSI->sphMarkersD2->velMasD,
                                       m_sysFSI->sphMarkersD2->rhoPresMuD, m_sysFSI->fsiGeneralData->referenceArray,
                                       outfilename);
    } else if (m_write_mode == OutpuMode::CHPF) {
        utils::WriteChPFParticlesToFile(m_sysFSI->sphMarkersD2->posRadD, m_sysFSI->fsiGeneralData->referenceArray,
                                        outfilename);
    }
}

void ChSystemFsi::PrintParticleToFile(const std::string& dir) const {
    utils::PrintParticleToFile(m_sysFSI->sphMarkersD2->posRadD, m_sysFSI->sphMarkersD2->velMasD,
                               m_sysFSI->sphMarkersD2->rhoPresMuD, m_sysFSI->fsiGeneralData->sr_tau_I_mu_i,
                               m_sysFSI->fsiGeneralData->derivVelRhoD, m_sysFSI->fsiGeneralData->referenceArray,
                               m_sysFSI->fsiGeneralData->referenceArray_FEA, dir, m_paramsH);
}

void ChSystemFsi::PrintFsiInfoToFile(const std::string& dir, double time) const {
    utils::PrintFsiInfoToFile(m_sysFSI->fsiBodiesD2->posRigid_fsiBodies_D,
                              m_sysFSI->fsiBodiesD2->velMassRigid_fsiBodies_D, m_sysFSI->fsiBodiesD2->q_fsiBodies_D,
                              m_sysFSI->fsiMeshD->pos_fsi_fea_D, m_sysFSI->fsiMeshD->vel_fsi_fea_D,
                              m_sysFSI->fsiGeneralData->rigid_FSI_ForcesD, m_sysFSI->fsiGeneralData->rigid_FSI_TorquesD,
                              m_sysFSI->fsiGeneralData->Flex_FSI_ForcesD, dir, time);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddSPHParticle(const ChVector<>& point,
                                 double rho0,
                                 double pres0,
                                 double mu0,
                                 const ChVector<>& velocity,
                                 const ChVector<>& tauXxYyZz,
                                 const ChVector<>& tauXyXzYz) {
    Real h = m_paramsH->HSML;
    m_sysFSI->AddSPHParticle(utils::ToReal4(point, h), mR4(rho0, pres0, mu0, -1),
                             utils::ToReal3(velocity),
                             utils::ToReal3(tauXxYyZz),
                             utils::ToReal3(tauXyXzYz));
}

void ChSystemFsi::AddSPHParticle(const ChVector<>& point,
                                 const ChVector<>& velocity,
                                 const ChVector<>& tauXxYyZz,
                                 const ChVector<>& tauXyXzYz) {
    AddSPHParticle(point, m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, velocity, tauXxYyZz, tauXyXzYz);
}

void ChSystemFsi::AddBoxSPH(const ChVector<>& boxCenter, const ChVector<>& boxHalfDim) {
    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(m_paramsH->INITSPACE);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        AddSPHParticle(points[i], m_paramsH->rho0, 0, m_paramsH->mu0,
                       ChVector<>(0),   // initial velocity
                       ChVector<>(0),   // tauxxyyzz
                       ChVector<>(0));  // tauxyxzyz
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddWallBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2<> size) {
    thrust::host_vector<Real4> bce;
    CreateBCE_wall(mR2(size.x(), size.y()), bce);
    AddBCE(body, bce, frame, false, false, false);
}

void ChSystemFsi::AddContainerBCE(std::shared_ptr<ChBody> body,
                                  const ChFrame<>& frame,
                                  const ChVector<>& size,
                                  const ChVector<int> faces) {
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    Real buffer = 2 * m_paramsH->NUM_BOUNDARY_LAYERS * spacing;

    // Wall center positions
    ChVector<> zn(0, 0, -spacing);
    ChVector<> zp(0, 0, size.z() + spacing);
    ChVector<> xn(-size.x() / 2 - spacing, 0, size.z() / 2);
    ChVector<> xp(+size.x() / 2 + spacing, 0, size.z() / 2);
    ChVector<> yn(0, -size.y() / 2 - spacing, size.z() / 2);
    ChVector<> yp(0, +size.y() / 2 + spacing, size.z() / 2);

    // Z- wall
    if (faces.z() == -1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zn, QUNIT), {size.x(), size.y()});
    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zp, Q_from_AngX(CH_C_PI)), {size.x(), size.y()});

    // X- wall
    if (faces.x() == -1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xn, Q_from_AngY(+CH_C_PI_2)), {size.z() + buffer, size.y()});
    // X+ wall
    if (faces.x() == +1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xp, Q_from_AngY(-CH_C_PI_2)), {size.z() + buffer, size.y()});

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yn, Q_from_AngX(-CH_C_PI_2)), {size.x() + buffer, size.z() + buffer});
    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yp, Q_from_AngX(+CH_C_PI_2)), {size.x() + buffer, size.z() + buffer});
}

void ChSystemFsi::AddBoxBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector<>& size, bool solid) {
    thrust::host_vector<Real4> bce;
    CreateBCE_box(utils::ToReal3(size), solid, bce);
    AddBCE(body, bce, frame, solid, false, false);
}

void ChSystemFsi::AddSphereBCE(std::shared_ptr<ChBody> body,
                               const ChFrame<>& frame,
                               double radius,
                               bool solid,
                               bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_sphere(radius, solid, polar, bce);
    AddBCE(body, bce, frame, solid, false, false);
}

void ChSystemFsi::AddCylinderBCE(std::shared_ptr<ChBody> body,
                                 const ChFrame<>& frame,
                                 double radius,
                                 double height,
                                 bool solid,
                                 bool capped,
                                 bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cylinder(radius, height, solid, capped, polar, bce);
    AddBCE(body, bce, frame, solid, false, false);
}

void ChSystemFsi::AddWheelBCE_Grouser(std::shared_ptr<ChBody> body,
                                 const ChFrame<>& frame,
                                 double radius,
                                 double wide,
                                 double grouser_height, 
                                 double grouser_wide, 
                                 int grouser_num,
                                 double kernel_h,
                                 bool cartesian) {
    thrust::host_vector<Real4> bce;
    CreateBCE_On_Wheel_Grouser(bce, radius, wide, grouser_height, grouser_wide, 
        grouser_num, m_paramsH, kernel_h, cartesian);
    AddBCE(body, bce, frame, true, false, false);
}

void ChSystemFsi::AddCylinderAnnulusBCE(std::shared_ptr<ChBody> body,
                                        const ChFrame<>& frame,
                                        double radius_inner,
                                        double radius_outer,
                                        double height,
                                        bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cylinder_annulus(radius_inner, radius_outer, height, polar, bce);
    AddBCE(body, bce, frame, true, false, false);
}

void ChSystemFsi::AddConeBCE(std::shared_ptr<ChBody> body,
                             const ChFrame<>& frame,
                             double radius,
                             double height,
                             bool solid,
                             bool capped,
                             bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cone(radius, height, solid, capped, polar, bce);
    AddBCE(body, bce, frame, solid, false, false);
}

void ChSystemFsi::AddPointsBCE(std::shared_ptr<ChBody> body,
                               const std::vector<ChVector<>>& points,
                               const ChFrame<>& frame,
                               bool solid) {
    thrust::host_vector<Real4> bce;
    for (const auto& p : points)
        bce.push_back(mR4(p.x(), p.y(), p.z(), m_paramsH->HSML));
    AddBCE(body, bce, frame, solid, false, false);
}

//// RADU TODO
void ChSystemFsi::AddFEAmeshBCE(std::shared_ptr<fea::ChMesh> my_mesh,
                                const std::vector<std::vector<int>>& NodeNeighborElement,
                                const std::vector<std::vector<int>>& _1D_elementsNodes,
                                const std::vector<std::vector<int>>& _2D_elementsNodes,
                                bool add1DElem,
                                bool add2DElem,
                                bool multiLayer,
                                bool removeMiddleLayer,
                                int SIDE,
                                int SIDE2D) {
    thrust::host_vector<Real4> posRadBCE;
    int numElems = my_mesh->GetNelements();
    std::vector<int> remove2D;
    std::vector<int> remove2D_s;
    std::vector<int> remove1D;

    for (size_t i = 0; i < my_mesh->GetNnodes(); i++) {
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(my_mesh->GetNode((unsigned int)i));
        m_fsi_interface->m_fsi_nodes.push_back(node);
    }

    for (size_t i = 0; i < numElems; i++) {
        // Check for Cable Elements
        if (_1D_elementsNodes.size() > 0) {
            if (auto thisCable =
                    std::dynamic_pointer_cast<fea::ChElementCableANCF>(my_mesh->GetElement((unsigned int)i))) {
                m_num_cable_elements++;

                remove1D.resize(2);
                std::fill(remove1D.begin(), remove1D.end(), 0);

                size_t myNumNodes = (_1D_elementsNodes[i].size() > 2) ? 2 : _1D_elementsNodes[i].size();
                for (size_t j = 0; j < myNumNodes; j++) {
                    int thisNode = _1D_elementsNodes[i][j];

                    // Look into the elements attached to thisNode
                    for (size_t k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        int neighborElement = NodeNeighborElement[thisNode][k];
                        if (neighborElement >= i)
                            continue;
                        remove1D[j] = 1;
                    }
                }

                if (add1DElem) {
                    CreateBCE_cable(posRadBCE, thisCable, remove1D, multiLayer, removeMiddleLayer, SIDE);
                    AddBCE_cable(posRadBCE, thisCable);
                }
                posRadBCE.clear();
            }
        }
        size_t Curr_size = _1D_elementsNodes.size();

        // Check for Shell Elements
        if (_2D_elementsNodes.size() > 0) {
            if (auto thisShell =
                    std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(my_mesh->GetElement((unsigned int)i))) {
                m_num_shell_elements++;

                remove2D.resize(4);
                remove2D_s.resize(4);
                std::fill(remove2D.begin(), remove2D.begin() + 4, 0);
                std::fill(remove2D_s.begin(), remove2D_s.begin() + 4, 0);

                // Look into the nodes of this element
                size_t myNumNodes =
                    (_2D_elementsNodes[i - Curr_size].size() > 4) ? 4 : _2D_elementsNodes[i - Curr_size].size();

                for (size_t j = 0; j < myNumNodes; j++) {
                    int thisNode = _2D_elementsNodes[i - Curr_size][j];
                    // Look into the elements attached to thisNode
                    for (size_t k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        // If this neighbor element has more than one common node with the previous
                        // node this means that we must not add BCEs to this edge anymore. Because
                        // that edge has already been given BCE particles.
                        // The kth element of this node:
                        size_t neighborElement = NodeNeighborElement[thisNode][k] - Curr_size;
                        if (neighborElement >= i - Curr_size)
                            continue;

                        size_t JNumNodes = (_2D_elementsNodes[neighborElement].size() > 4)
                                               ? 4
                                               : _2D_elementsNodes[neighborElement].size();

                        for (size_t inode = 0; inode < myNumNodes; inode++) {
                            for (size_t jnode = 0; jnode < JNumNodes; jnode++) {
                                if (_2D_elementsNodes[i - Curr_size][inode] ==
                                        _2D_elementsNodes[neighborElement][jnode] &&
                                    thisNode != _2D_elementsNodes[i - Curr_size][inode] && i > neighborElement) {
                                    remove2D[inode] = 1;
                                    if (inode == j + 1 || j > inode + 1) {
                                        remove2D_s[j] = 1;
                                    } else {
                                        remove2D_s[inode] = 1;
                                    }
                                }
                            }
                        }
                    }
                }

                if (add2DElem) {
                    CreateBCE_shell(posRadBCE, thisShell, remove2D, remove2D_s, multiLayer, removeMiddleLayer, SIDE2D);
                    AddBCE_shell(posRadBCE, thisShell);
                }
                posRadBCE.clear();
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

const Real pi = Real(CH_C_PI);

void ChSystemFsi::CreateBCE_wall(const Real2& size, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing in x-y directions
    Real2 hsize = size / 2;
    int2 np = {(int)std::round(hsize.x / spacing), (int)std::round(hsize.y / spacing)};
    Real2 delta = {hsize.x / np.x, hsize.y / np.y};

    for (int il = 0; il < num_layers; il++) {
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back(mR4(ix * delta.x, iy * delta.y, -il * spacing, kernel_h));
            }
        }
    }
}

void ChSystemFsi::CreateBCE_box(const Real3& size, bool solid, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing in all 3 directions
    Real3 hsize = size / 2;
    int3 np = {(int)std::round(hsize.x / spacing), (int)std::round(hsize.y / spacing),
               (int)std::round(hsize.z / spacing)};
    Real3 delta = {hsize.x / np.x, hsize.y / np.y, hsize.z / np.z};

    // Inflate box if boundary
    if (!solid) {
        np += num_layers - 1;
        hsize = hsize + (num_layers - 1) * delta;
    }

    for (int il = 0; il < num_layers; il++) {
        // faces in Z direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back(mR4(ix * delta.x, iy * delta.y, -hsize.z + il * delta.z, kernel_h));
                bce.push_back(mR4(ix * delta.x, iy * delta.y, +hsize.z - il * delta.z, kernel_h));
            }
        }

        // faces in Y direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back(mR4(ix * delta.x, -hsize.y + il * delta.y, iz * delta.z, kernel_h));
                bce.push_back(mR4(ix * delta.x, +hsize.y - il * delta.y, iz * delta.z, kernel_h));
            }
        }

        // faces in X direction
        for (int iy = -np.y + num_layers; iy <= np.y - num_layers; iy++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back(mR4(-hsize.x + il * delta.x, iy * delta.y, iz * delta.z, kernel_h));
                bce.push_back(mR4(+hsize.x - il * delta.x, iy * delta.y, iz * delta.z, kernel_h));
            }
        }
    }
}

void ChSystemFsi::CreateBCE_sphere(Real rad, bool solid, bool polar, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Use polar coordinates
    if (polar) {
        Real rad_out = solid ? rad : rad + num_layers * spacing;
        Real rad_in = rad_out - num_layers * spacing;
        int np_r = (int)std::round((rad - rad_in) / spacing);
        Real delta_r = (rad_out - rad_in) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_in + ir * delta_r;
            int np_phi = (int)std::round(pi * r / spacing);
            Real delta_phi = pi / np_phi;
            for (int ip = 0; ip < np_phi; ip++) {
                Real phi = ip * delta_phi;
                Real cphi = std::cos(phi);
                Real sphi = std::sin(phi);
                Real x = r * sphi;
                Real y = r * sphi;
                Real z = r * cphi;
                int np_th = (int)std::round(2 * pi * r * sphi / spacing);
                Real delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    bce.push_back(mR4(x * std::cos(theta), y * std::sin(theta), z, kernel_h));
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np = (int)std::round(rad / spacing);
    Real delta = rad / np;
    if (!solid) {
        np += num_layers;
        rad += num_layers * delta;
    }

    for (int iz = 0; iz <= np; iz++) {
        Real z = iz * delta;
        Real rz_max = std::sqrt(rad * rad - z * z);
        Real rz_min = std::max(rz_max - num_layers * delta, Real(0.0));
        if (iz >= np - num_layers)
            rz_min = 0;
        Real rz_min2 = rz_min * rz_min;
        Real rz_max2 = rz_max * rz_max;
        int nq = (int)std::round(rz_max / spacing);
        for (int ix = -nq; ix <= nq; ix++) {
            Real x = ix * delta;
            for (int iy = -nq; iy <= nq; iy++) {
                Real y = iy * delta;
                Real r2 = x * x + y * y;
                if (r2 >= rz_min2 && r2 <= rz_max2) {
                    bce.push_back(mR4(x, y, +z, kernel_h));
                    bce.push_back(mR4(x, y, -z, kernel_h));
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cylinder(Real rad,
                                     Real height,
                                     bool solid,
                                     bool capped,
                                     bool polar,
                                     thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / spacing);
    Real delta_h = hheight / np_h;

    // Inflate cylinder if boundary
    if (!solid && capped) {
        np_h += num_layers;
        hheight += num_layers * delta_h;
    }

    // Use polar coordinates
    if (polar) {
        Real rad_max = solid ? rad : rad + num_layers * spacing;
        Real rad_min = rad_max - num_layers * spacing;
        int np_r = (int)std::round((rad_max - rad_min) / spacing);
        Real delta_r = (rad_max - rad_min) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_min + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            Real delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
            for (int it = 0; it < np_th; it++) {
                Real theta = it * delta_th;
                Real x = r * cos(theta);
                Real y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }

        if (capped) {
            rad_max = rad_min - delta_r;
            np_r = (int)std::round(rad_max / spacing);
            delta_r = rad_max / np_r;

            for (int ir = 0; ir <= np_r; ir++) {
                Real r = rad_max - ir * delta_r;
                int np_th = std::max((int)std::round(2 * pi * r / spacing), 1);
                Real delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    Real x = r * cos(theta);
                    Real y = r * sin(theta);
                    for (int iz = 0; iz <= num_layers; iz++) {
                        Real z = hheight - iz * delta_h;
                        bce.push_back(mR4(x, y, -z, kernel_h));
                        bce.push_back(mR4(x, y, +z, kernel_h));
                    }
                }
            }
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    Real delta_r = rad / np_r;
    if (!solid) {
        np_r += num_layers;
        rad += num_layers * delta_r;
    }

    Real rad_max = rad;
    Real rad_min = std::max(rad - num_layers * delta_r, Real(0.0));
    Real r_max2 = rad_max * rad_max;
    Real r_min2 = rad_min * rad_min;
    for (int ix = -np_r; ix <= np_r; ix++) {
        Real x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            Real y = iy * delta_r;
            Real r2 = x * x + y * y;
            if (r2 >= r_min2 && r2 <= r_max2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
            if (capped && r2 < r_min2) {
                for (int iz = 0; iz <= num_layers; iz++) {
                    Real z = hheight - iz * delta_h;
                    bce.push_back(mR4(x, y, -z, kernel_h));
                    bce.push_back(mR4(x, y, +z, kernel_h));              
                }                
            }
        }
    }
}


void ChSystemFsi::CreateBCE_On_Wheel_Grouser(
                           thrust::host_vector<Real4>& posRadBCE,
                           Real wheel_rad,
                           Real wheel_w,
                           Real gro_h,
                           Real gro_w,
                           int gro_num,
                           std::shared_ptr<SimParams> paramsH,
                           Real kernel_h,
                           bool cartesian) {
    Real spacing = kernel_h * paramsH->MULT_INITSPACE;
    int num_layers = (int)std::floor(1.00001 * wheel_w / spacing) + 1;
    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * wheel_w + spacing * si;
        if (cartesian)
            for (Real x = -wheel_rad; x <= wheel_rad; x += spacing) {
                for (Real y = -wheel_rad; y <= wheel_rad; y += spacing) {
                    if (x * x + y * y <= wheel_rad * wheel_rad)
                        posRadBCE.push_back(mR4(x, s, y, kernel_h));
                }
            }
        else {
            Real3 centerPointLF = mR3(0, s, 0);
            posRadBCE.push_back(mR4(0, s, 0, kernel_h));
            // wheel
            int numr = (int)std::floor(1.00001 * wheel_rad / spacing);
            for (size_t ir = 0; ir < numr; ir++) {
                Real r = spacing + ir * spacing;
                int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
                for (size_t t = 0; t < numTheta; t++) {
                    Real teta = t * 2 * 3.1415 / numTheta;
                    Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                    posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                }
            }
            // grouser
            int numr_g = (int)std::floor(1.00001 * gro_h / spacing);
            int numw_g = (int)std::floor(1.00001 * gro_w / spacing) + 1;
            for (size_t ir_g = 0; ir_g < numr_g; ir_g++) {
                Real r = 0.5 * spacing + ir_g * spacing + wheel_rad;
                for (size_t t = 0; t < gro_num; t++) {
                    for (size_t iw_g = 0; iw_g < numw_g; iw_g++) {
                        Real teta = t * 2 * 3.1415 / gro_num + iw_g * spacing / wheel_rad;
                        Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                        posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                    }
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cylinder_annulus(Real rad_in,
                                             Real rad_out,
                                             Real height,
                                             bool polar,
                                             thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;

    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / spacing);
    Real delta_h = hheight / np_h;

    // Use polar coordinates
    if (polar) {
        int np_r = (int)std::round((rad_out - rad_in) / spacing);
        Real delta_r = (rad_out - rad_in) / np_r;
        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_in + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            Real delta_th = (2 * pi) / np_th;
            for (int it = 0; it < np_th; it++) {
                Real theta = it * delta_th;
                Real x = r * cos(theta);
                Real y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad_out / spacing);
    Real delta_r = rad_out / np_r;

    Real r_in2 = rad_in * rad_in;
    Real r_out2 = rad_out * rad_out;
    for (int ix = -np_r; ix <= np_r; ix++) {
        Real x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            Real y = iy * delta_r;
            Real r2 = x * x + y * y;
            if (r2 >= r_in2 && r2 <= r_out2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cone(Real rad,
                                 Real height,
                                 bool solid,
                                 bool capped,
                                 bool polar,
                                 thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing
    int np_h = (int)std::round(height / spacing);
    Real delta_h = height / np_h;

    // Inflate cone if boundary
    if (!solid) {
        np_h += num_layers;
        height += num_layers * delta_h;
        if (capped) {
            np_h += num_layers;
            height += num_layers * delta_h;
        }
    }

    // Use polar coordinates
    if (polar) {
        for (int iz = 0; iz < np_h; iz++) {
            Real z = iz * delta_h;
            Real rz = rad * (height - z) / height;
            Real rad_out = solid ? rz : rz + num_layers * spacing;
            Real rad_in = std::max(rad_out - num_layers * spacing, Real(0.0));
            if (iz >= np_h - num_layers)
                rad_in = 0;
            int np_r = (int)std::round((rad_out - rad_in) / spacing);
            Real delta_r = (rad_out - rad_in) / np_r;
            for (int ir = 0; ir <= np_r; ir++) {
                Real r = rad_in + ir * delta_r;
                int np_th = (int)std::round(2 * pi * r / spacing);
                Real delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    Real x = r * cos(theta);
                    Real y = r * sin(theta);
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }

        bce.push_back(mR4(0.0, 0.0, height, kernel_h));

        if (capped) {
            //// RADU TODO
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    Real delta_r = rad / np_r;

    for (int iz = 0; iz <= np_h; iz++) {
        Real z = iz * delta_h;
        Real rz = rad * (height - z) / height;
        Real rad_out = solid ? rz : rz + num_layers * spacing;
        Real rad_in = std::max(rad_out - num_layers * spacing, Real(0.0));
        Real r_out2 = rad_out * rad_out;
        Real r_in2 = rad_in * rad_in;
        for (int ix = -np_r; ix <= np_r; ix++) {
            Real x = ix * delta_r;
            for (int iy = -np_r; iy <= np_r; iy++) {
                Real y = iy * delta_r;
                Real r2 = x * x + y * y;
                if (r2 >= r_in2 && r2 <= r_out2) {
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }

            if (capped) {
                //// RADU TODO
            }
        }
    }
}

//// RADU TODO
void ChSystemFsi::CreateBCE_cable(thrust::host_vector<Real4>& posRadBCE,
                                  std::shared_ptr<chrono::fea::ChElementCableANCF> cable,
                                  std::vector<int> remove,
                                  bool multiLayer,
                                  bool removeMiddleLayer,
                                  int SIDE) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;

    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();
    double nX = dx / spacing - std::floor(dx / spacing);
    int nFX = (int)std::floor(dx / spacing);
    if (nX > 0.5)
        nFX++;

    Real initSpaceX;
    if (nFX != 0)
        initSpaceX = dx / nFX;
    else
        initSpaceX = dx;

    Real initSpaceZ = spacing;
    int2 iBound = mI2(0, nFX);

    for (int i = iBound.x; i <= iBound.y; i++) {
        bool con1 = (remove[1] && (i == iBound.y));
        bool con2 = (remove[0] && (i == iBound.x));
        if (con1 || con2)
            continue;

        Real3 relMarkerPos;
        double CONSTANT = 1.0;
        if (multiLayer) {
            for (int j = 1; j <= SIDE; j++) {
                relMarkerPos = mR3(i * initSpaceX, j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, -j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, 0, j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, 0, -j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
            }
        }

        if (!removeMiddleLayer) {
            relMarkerPos = mR3(i * initSpaceX, 0, 0);
            posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
        }
    }
}

//// RADU TODO
void ChSystemFsi::CreateBCE_shell(thrust::host_vector<Real4>& posRadBCE,
                                  std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                                  std::vector<int> remove,
                                  std::vector<int> remove_s,
                                  bool multiLayer,
                                  bool removeMiddleLayer,
                                  int SIDE) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;

    double dx = shell->GetLengthX() / 2;
    double dy = shell->GetLengthY() / 2;

    double nX = dx / spacing - std::floor(dx / spacing);
    double nY = dy / spacing - std::floor(dy / spacing);
    int nFX = (int)std::floor(dx / spacing);
    int nFY = (int)std::floor(dy / spacing);
    if (nX > 0.5)
        nFX++;
    if (nY > 0.5)
        nFY++;

    Real initSpaceX;
    Real initSpaceY;
    int2 iBound;
    int2 jBound;

    if (dx < nFX * spacing) {
        iBound = mI2(-nFX * 2, nFX * 2);
        initSpaceX = dx / nFX;
    } else {
        iBound = mI2(-nFX * 2 - 1, nFX * 2 + 1);
        initSpaceX = dx / (0.5 + nFX);
    }

    if (dy < nFY * spacing) {
        jBound = mI2(-nFY * 2, nFY * 2);
        initSpaceY = dy / nFY;
    } else {
        jBound = mI2(-nFY * 2 - 1, nFY * 2 + 1);
        initSpaceY = dy / (0.5 + nFY);
    }

    int2 kBound;
    // If multi-layer BCE is required
    if (SIDE > 0 && multiLayer)  // Do SIDE number layers in one side
        kBound = mI2(0, SIDE);
    else if (SIDE < 0 && multiLayer)  // Do SIDE number layers in the other side
        kBound = mI2(SIDE, 0);
    else if (SIDE == 0 && multiLayer)  // Do 1 layer on each side. Note that there would be 3 layers in total
        kBound = mI2(-1, 1);           // The middle layer would be on the shell
    else                               // IF you do not want multi-layer just use one layer on the shell
        kBound = mI2(0, 0);            // This will create some marker deficiency and reduce the accuracy but look nicer

    for (int k = kBound.x; k <= kBound.y; k++) {
        //// RADU TODO
        ////    There should be no side-effect in this function!!!!!
        ////if (k == 0 && SIDE == 0 && multiLayer && removeMiddleLayer) {
        ////    // skip the middle layer for this specific case
        ////    // change value of paramsH->MULT_INITSPACE_Shells
        ////    m_paramsH->MULT_INITSPACE_Shells = 0.5;
        ////    continue;
        ////}
        for (int j = jBound.x; j <= jBound.y; j = j + 2) {
            for (int i = iBound.x; i <= iBound.y; i = i + 2) {
                Real3 relMarkerPos = mR3(i * initSpaceX / 2.0, j * initSpaceY / 2.0, k);

                // It has to skip puting BCE on the nodes if one of the following conditions is true
                bool con1 = (remove_s[0] && j == jBound.x);
                bool con2 = (remove_s[2] && j == jBound.y);
                bool con3 = (remove_s[1] && i == iBound.y);
                bool con4 = (remove_s[3] && i == iBound.x);
                bool con5 =
                    (remove[0] && remove[1] && (!remove_s[0]) && j == jBound.x && (i == iBound.x || i == iBound.y));
                bool con6 =
                    (remove[2] && remove[3] && (!remove_s[2]) && j == jBound.y && (i == iBound.x || i == iBound.y));
                bool con7 =
                    (remove[1] && remove[2] && (!remove_s[1]) && i == iBound.y && (j == jBound.x || j == jBound.y));
                bool con8 =
                    (remove[3] && remove[0] && (!remove_s[3]) && i == iBound.x && (j == jBound.x || j == jBound.y));

                if (con1 || con2 || con3 || con4 || con5 || con6 || con7 || con8)
                    continue;

                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddBCE(std::shared_ptr<ChBody> body,
                         const thrust::host_vector<Real4>& bce,
                         const ChFrame<>& rel_frame,
                         bool solid,
                         bool add_to_fluid_helpers,
                         bool add_to_previous) {
    // Set BCE marker type
    int type = 0;
    if (solid)
        type = 1;
    if (add_to_fluid_helpers)
        type = -3;

    for (const auto& p : bce) {
        auto pos_shape = utils::ToChVector(p);
        auto pos_body = rel_frame.TransformPointLocalToParent(pos_shape);
        auto pos_abs = body->GetFrame_REF_to_abs().TransformPointLocalToParent(pos_body);
        auto vel_abs = body->GetFrame_REF_to_abs().PointSpeedLocalToParent(pos_body);

        m_sysFSI->sphMarkersH->posRadH.push_back(mR4(utils::ToReal3(pos_abs), p.w));
        m_sysFSI->sphMarkersH->velMasH.push_back(utils::ToReal3(vel_abs));

        m_sysFSI->sphMarkersH->rhoPresMuH.push_back(
            mR4(m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, (double)type));
        m_sysFSI->sphMarkersH->tauXxYyZzH.push_back(mR3(0.0));
        m_sysFSI->sphMarkersH->tauXyXzYzH.push_back(mR3(0.0));
    }

    if (solid)
        m_fsi_bodies_bce_num.push_back((int)bce.size());
}

//// RADU TODO
void ChSystemFsi::AddBCE_cable(const thrust::host_vector<Real4>& posRadBCE,
                               std::shared_ptr<fea::ChElementCableANCF> cable) {
    int type = 2;

    fea::ChElementCableANCF::ShapeVector N;
    fea::ChElementCableANCF::ShapeVector Nd;

    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();
    ChVector<> physic_to_natural(1 / dx, 1, 1);

    ChVector<> nAp = cable->GetNodeA()->GetPos();
    ChVector<> nBp = cable->GetNodeB()->GetPos();

    ChVector<> nAv = cable->GetNodeA()->GetPos_dt();
    ChVector<> nBv = cable->GetNodeB()->GetPos_dt();

    ChVector<> nAdir = cable->GetNodeA()->GetD();
    ChVector<> nBdir = cable->GetNodeB()->GetD();

    ChVector<> nAdirv = cable->GetNodeA()->GetD_dt();
    ChVector<> nBdirv = cable->GetNodeB()->GetD_dt();

    int posRadSizeModified = 0;
    if (m_verbose)
        printf(" posRadBCE.size()= :%zd\n", posRadBCE.size());

    for (size_t i = 0; i < posRadBCE.size(); i++) {
        ChVector<> pos_physical = utils::ToChVector(mR3(posRadBCE[i]));
        ChVector<> pos_natural = pos_physical * physic_to_natural;

        cable->ShapeFunctionsDerivatives(Nd, pos_natural.x());
        ChVector<> Element_Axis = Nd(0) * nAp + Nd(1) * nAdir + Nd(2) * nBp + Nd(3) * nBdir;
        Element_Axis.Normalize();

        ChVector<> new_y_axis = ChVector<>(-Element_Axis.y(), Element_Axis.x(), 0) +
                                ChVector<>(-Element_Axis.z(), 0, Element_Axis.x()) +
                                ChVector<>(0, -Element_Axis.z(), Element_Axis.y());
        new_y_axis.Normalize();
        ChVector<> new_z_axis = Vcross(Element_Axis, new_y_axis);

        cable->ShapeFunctions(N, pos_natural.x());
        ChVector<> Correct_Pos = N(0) * nAp + N(1) * nAdir + N(2) * nBp + N(3) * nBdir + new_y_axis * pos_physical.y() +
                                 new_z_axis * pos_physical.z();

        if ((Correct_Pos.x() < m_paramsH->cMin.x || Correct_Pos.x() > m_paramsH->cMax.x) ||
            (Correct_Pos.y() < m_paramsH->cMin.y || Correct_Pos.y() > m_paramsH->cMax.y) ||
            (Correct_Pos.z() < m_paramsH->cMin.z || Correct_Pos.z() > m_paramsH->cMax.z))
            continue;

        // Note that the fluid particles are removed differently
        bool addthis = true;
        for (size_t p = 0; p < m_sysFSI->sphMarkersH->posRadH.size() - 1; p++) {
            // Only compare to rigid and flexible BCE particles added previously
            if (m_sysFSI->sphMarkersH->rhoPresMuH[p].w > 0.5) {
                double dis =
                    length(mR3(m_sysFSI->sphMarkersH->posRadH[p]) - utils::ToReal3(Correct_Pos));
                if (dis < 1e-8) {
                    addthis = false;
                    if (m_verbose)
                        printf(" Already added a BCE particle here! Skip this one!\n");
                    break;
                }
            }
        }

        if (addthis) {
            m_sysFSI->sphMarkersH->posRadH.push_back(
                mR4(utils::ToReal3(Correct_Pos), posRadBCE[i].w));
            m_sysFSI->fsiGeneralData->FlexSPH_MeshPos_LRF_H.push_back(utils::ToReal3(pos_natural));
            ChVector<> Correct_Vel = N(0) * nAv + N(1) * nAdirv + N(2) * nBv + N(3) * nBdirv + ChVector<double>(1e-20);
            Real3 v3 = utils::ToReal3(Correct_Vel);
            m_sysFSI->sphMarkersH->velMasH.push_back(v3);
            m_sysFSI->sphMarkersH->rhoPresMuH.push_back(
                mR4(m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, type));
            posRadSizeModified++;
        }
    }
    m_fsi_cables_bce_num.push_back(posRadSizeModified);
}

//// RADU TODO
void ChSystemFsi::AddBCE_shell(const thrust::host_vector<Real4>& posRadBCE,
                               std::shared_ptr<fea::ChElementShellANCF_3423> shell) {
    int type = 3;
    fea::ChElementShellANCF_3423::ShapeVector N;
    int posRadSizeModified = 0;

    double my_h = m_paramsH->HSML;

    Real dx = shell->GetLengthX();
    Real dy = shell->GetLengthY();
    ChVector<> physic_to_natural(2 / dx, 2 / dy, 1);
    ChVector<> nAp = shell->GetNodeA()->GetPos();
    ChVector<> nBp = shell->GetNodeB()->GetPos();
    ChVector<> nCp = shell->GetNodeC()->GetPos();
    ChVector<> nDp = shell->GetNodeD()->GetPos();

    ChVector<> nAdir = shell->GetNodeA()->GetD();
    ChVector<> nBdir = shell->GetNodeB()->GetD();
    ChVector<> nCdir = shell->GetNodeC()->GetD();
    ChVector<> nDdir = shell->GetNodeD()->GetD();

    ChVector<> nAv = shell->GetNodeA()->GetPos_dt();
    ChVector<> nBv = shell->GetNodeB()->GetPos_dt();
    ChVector<> nCv = shell->GetNodeC()->GetPos_dt();
    ChVector<> nDv = shell->GetNodeD()->GetPos_dt();

    if (m_verbose)
        printf(" posRadBCE.size()= :%zd\n", posRadBCE.size());

    for (size_t i = 0; i < posRadBCE.size(); i++) {
        ChVector<> pos_physical = utils::ToChVector(mR3(posRadBCE[i]));
        ChVector<> pos_natural = pos_physical * physic_to_natural;

        shell->ShapeFunctions(N, pos_natural.x(), pos_natural.y(), pos_natural.z());

        ChVector<> Normal = N(0) * nAdir + N(2) * nBdir + N(4) * nCdir + N(6) * nDdir;
        Normal.Normalize();

        ChVector<> Correct_Pos = N(0) * nAp + N(2) * nBp + N(4) * nCp + N(6) * nDp +
                                 Normal * pos_physical.z() * my_h * m_paramsH->MULT_INITSPACE_Shells;

        if ((Correct_Pos.x() < m_paramsH->cMin.x || Correct_Pos.x() > m_paramsH->cMax.x) ||
            (Correct_Pos.y() < m_paramsH->cMin.y || Correct_Pos.y() > m_paramsH->cMax.y) ||
            (Correct_Pos.z() < m_paramsH->cMin.z || Correct_Pos.z() > m_paramsH->cMax.z))
            continue;

        // Note that the fluid particles are removed differently
        bool addthis = true;
        for (size_t p = 0; p < m_sysFSI->sphMarkersH->posRadH.size() - 1; p++) {
            // Only compare to rigid and flexible BCE particles added previously
            if (m_sysFSI->sphMarkersH->rhoPresMuH[p].w > 0.5) {
                double dis =
                    length(mR3(m_sysFSI->sphMarkersH->posRadH[p]) - utils::ToReal3(Correct_Pos));
                if (dis < 1e-8) {
                    addthis = false;
                    if (m_verbose)
                        printf(" Already added a BCE particle here! Skip this one!\n");
                    break;
                }
            }
        }

        if (addthis) {
            m_sysFSI->sphMarkersH->posRadH.push_back(
                mR4(utils::ToReal3(Correct_Pos), posRadBCE[i].w));
            m_sysFSI->fsiGeneralData->FlexSPH_MeshPos_LRF_H.push_back(utils::ToReal3(pos_natural));

            ChVector<> Correct_Vel = N(0) * nAv + N(2) * nBv + N(4) * nCv + N(6) * nDv;
            Real3 v3 = utils::ToReal3(Correct_Vel);
            m_sysFSI->sphMarkersH->velMasH.push_back(v3);
            m_sysFSI->sphMarkersH->rhoPresMuH.push_back(
                mR4(m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, type));
            posRadSizeModified++;
        }
    }
    m_fsi_shells_bce_num.push_back(posRadSizeModified);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CreateMeshPoints(geometry::ChTriangleMeshConnected& mesh,
                                   double delta,
                                   std::vector<ChVector<>>& point_cloud) {
    mesh.RepairDuplicateVertexes(1e-9);  // if meshes are not watertight
    ChVector<> minV;
    ChVector<> maxV;
    mesh.GetBoundingBox(minV, maxV, ChMatrix33<>(1));

    const double EPSI = 1e-6;

    ChVector<> ray_origin;
    for (double x = minV.x(); x < maxV.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = minV.y(); y < maxV.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = minV.z(); z < maxV.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector<> ray_dir[2] = {ChVector<>(5, 0.5, 0.25), ChVector<>(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh.m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh.m_face_v_indices[i];
                    auto& v1 = mesh.m_vertices[t_face.x()];
                    auto& v2 = mesh.m_vertices[t_face.y()];
                    auto& v3 = mesh.m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    int t_inter[2] = {0, 0};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = 0;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        /// Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = 0;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = 0;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  /// ray intersection
                            t_inter[j] = 1;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = 0;
                    }

                    intersectCounter[0] += t_inter[0];
                    intersectCounter[1] += t_inter[1];
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    point_cloud.push_back(ChVector<>(x, y, z));
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

double ChSystemFsi::GetKernelLength() const {
    return m_paramsH->HSML;
}

double ChSystemFsi::GetInitialSpacing() const {
    return m_paramsH->INITSPACE;
}

ChVector<> ChSystemFsi::GetContainerDim() const {
    return ChVector<>(m_paramsH->boxDimX, m_paramsH->boxDimY, m_paramsH->boxDimZ);
}

double ChSystemFsi::GetDensity() const {
    return m_paramsH->rho0;
}

double ChSystemFsi::GetViscosity() const {
    return m_paramsH->mu0;
}

double ChSystemFsi::GetBasePressure() const {
    return m_paramsH->BASEPRES;
}

double ChSystemFsi::GetParticleMass() const {
    return m_paramsH->markerMass;
}

ChVector<> ChSystemFsi::Get_G_acc() const {
    return ChVector<>(m_paramsH->gravity.x, m_paramsH->gravity.y, m_paramsH->gravity.z);
}

double ChSystemFsi::GetSoundSpeed() const {
    return m_paramsH->Cs;
}

ChVector<> ChSystemFsi::GetBodyForce() const {
    return ChVector<>(m_paramsH->bodyForce3.x, m_paramsH->bodyForce3.y, m_paramsH->bodyForce3.z);
}

double ChSystemFsi::GetStepSize() const {
    return m_paramsH->dT;
}

double ChSystemFsi::GetMaxStepSize() const {
    return m_paramsH->dT_Max;
}

bool ChSystemFsi::GetAdaptiveTimeStepping() const {
    return m_paramsH->Adaptive_time_stepping;
}

size_t ChSystemFsi::GetNumFluidMarkers() const {
    return m_sysFSI->numObjects->numFluidMarkers;
}

size_t ChSystemFsi::GetNumRigidBodyMarkers() const {
    return m_sysFSI->numObjects->numRigidMarkers;
}

size_t ChSystemFsi::GetNumFlexBodyMarkers() const {
    return m_sysFSI->numObjects->numFlexMarkers;
}

size_t ChSystemFsi::GetNumBoundaryMarkers() const {
    return m_sysFSI->numObjects->numBoundaryMarkers;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<std::shared_ptr<ChBody>>& ChSystemFsi::GetFsiBodies() const {
    return m_fsi_interface->m_fsi_bodies;
}

std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& ChSystemFsi::GetFsiNodes() const {
    return m_fsi_interface->m_fsi_nodes;
}

std::shared_ptr<fea::ChMesh> ChSystemFsi::GetFsiMesh() const {
    return m_fsi_interface->m_fsi_mesh;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<ChVector<>> ChSystemFsi::GetParticlePositions() const {
    thrust::host_vector<Real4> posRadH = m_sysFSI->sphMarkersD2->posRadD;
    std::vector<ChVector<>> pos;
    for (size_t i = 0; i < posRadH.size(); i++) {
        pos.push_back(utils::ToChVector(posRadH[i]));
    }
    return pos;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleFluidProperties() const {
    thrust::host_vector<Real4> rhoPresMuH = m_sysFSI->sphMarkersD2->rhoPresMuD;
    std::vector<ChVector<>> props;
    for (size_t i = 0; i < rhoPresMuH.size(); i++) {
        props.push_back(utils::ToChVector(rhoPresMuH[i]));
    }
    return props;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleVelocities() const {
    thrust::host_vector<Real3> velH = m_sysFSI->sphMarkersD2->velMasD;
    std::vector<ChVector<>> vel;
    for (size_t i = 0; i < velH.size(); i++) {
        vel.push_back(utils::ToChVector(velH[i]));
    }
    return vel;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleAccelerations() const {
    thrust::host_vector<Real4> accH = m_sysFSI->GetParticleAccelerations();
    std::vector<ChVector<>> acc;
    for (size_t i = 0; i < accH.size(); i++) {
        acc.push_back(utils::ToChVector(accH[i]));
    }
    return acc;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleForces() const {
    thrust::host_vector<Real4> frcH = m_sysFSI->GetParticleForces();
    std::vector<ChVector<>> frc;
    for (size_t i = 0; i < frcH.size(); i++) {
        frc.push_back(utils::ToChVector(frcH[i]));
    }
    return frc;
}

//--------------------------------------------------------------------------------------------------------------------------------

thrust::device_vector<int> ChSystemFsi::FindParticlesInBox(const ChFrame<>& frame, const ChVector<>& size) {
    const ChVector<>& Pos = frame.GetPos();
    ChVector<> Ax = frame.GetA().Get_A_Xaxis();
    ChVector<> Ay = frame.GetA().Get_A_Yaxis();
    ChVector<> Az = frame.GetA().Get_A_Zaxis();

    auto hsize = 0.5 * mR3(size.x(), size.y(), size.z());
    auto pos = mR3(Pos.x(), Pos.y(), Pos.z());
    auto ax = mR3(Ax.x(), Ax.y(), Ax.z());
    auto ay = mR3(Ay.x(), Ay.y(), Ay.z());
    auto az = mR3(Az.x(), Az.y(), Az.z());

    return m_sysFSI->FindParticlesInBox(hsize, pos, ax, ay, az);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticlePositions(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticlePositions(indices);
}

thrust::device_vector<Real3> ChSystemFsi::GetParticleVelocities(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleVelocities(indices);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticleForces(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleForces(indices);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticleAccelerations(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleAccelerations(indices);
}

}  // end namespace fsi
}  // end namespace chrono
