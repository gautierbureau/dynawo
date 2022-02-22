//
// Copyright (c) 2022, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools
// for power systems.
//

/**
 * @file Models/CPP/Controls/Voltage/ModelVoltageMeasurementsUtilities/test/TestVoltageMeasurementsUtilities.cpp
 * @brief Unit tests for ModelVoltageMeasurementsUtilities model
 *
 */

#include <boost/shared_ptr.hpp>

#include "DYNModelVoltageMeasurementsUtilities.h"
#include "DYNModelVoltageMeasurementsUtilities.hpp"
#include "DYNVariable.h"
#include "PARParametersSet.h"
#include "DYNParameterModeler.h"
#include "DYNSparseMatrix.h"
#include "DYNElement.h"

#include "gtest_dynawo.h"

namespace DYN {

static boost::shared_ptr<SubModel> createModelVoltageMeasurementsUtilities(unsigned int nbVoltages_ = 0, double step_ = 12.0) {
    // All changes here should be adapted in the test for DefineMethods.
    boost::shared_ptr<SubModel> voltmu =
        SubModelFactory::createSubModelFromLib("../DYNModelVoltageMeasurementsUtilities" + std::string(sharedLibraryExtension()));
    voltmu->init(0.);  // Harmless coverage
    std::vector<ParameterModeler> parameters;
    voltmu->defineParameters(parameters);  // stands for VOLTage Measurement Utilities
    boost::shared_ptr<parameters::ParametersSet> parametersSet = boost::shared_ptr<parameters::ParametersSet>(new parameters::ParametersSet("Parameterset"));
    parametersSet->createParameter("nbInputs", static_cast<int>(nbVoltages_));
    parametersSet->createParameter("updateStep", step_);
    voltmu->setPARParameters(parametersSet);
    voltmu->addParameters(parameters, false);
    voltmu->setParametersFromPARFile();
    voltmu->setSubModelParameters();
    voltmu->getSize();  // Sets all the sizes
    return voltmu;
}

static void setBuffersVMU(boost::shared_ptr<SubModel> voltmu, propertyContinuousVar_t* yTypes,
                            double* voltages, double* z, bool* zConnected, state_g* g) {
// static void setBuffersVMU(boost::shared_ptr<SubModel> voltmu, std::vector<propertyContinuousVar_t> yTypes,
                            // std::vector<double> voltages, std::vector<double> z, bool* zConnected, std::vector<state_g> g) {
    voltmu->setBufferYType(&yTypes[0], 0);
    voltmu->evalStaticYType();

    voltmu->evalStaticFType();  // Does nothing here.
    voltmu->initializeFromData(boost::shared_ptr<DataInterface>());

    voltmu->setBufferZ(&z[0], zConnected, 0);
    voltmu->setBufferY(&voltages[0], nullptr, 0);
    voltmu->initializeStaticData();
    voltmu->evalDynamicFType();
    voltmu->evalDynamicYType();

    voltmu->getY0();
    voltmu->setBufferG(&g[0], 0);
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesDefineMethods) {
    // Checks all operations required in the general initModel function up there.
    boost::shared_ptr<SubModel> voltmu =
        SubModelFactory::createSubModelFromLib("../DYNModelVoltageMeasurementsUtilities" + std::string(sharedLibraryExtension()));
    voltmu->init(0.);
    std::vector<ParameterModeler> parameters;
    voltmu->defineParameters(parameters);  // stands for VOLTage Measurement Utilities
    ASSERT_EQ(parameters.size(), 2);
    // Define the model parameters
    double step = 12;
    unsigned int nbVoltages = 2;
    boost::shared_ptr<parameters::ParametersSet> parametersSet = boost::shared_ptr<parameters::ParametersSet>(new parameters::ParametersSet("Parameterset"));
    parametersSet->createParameter("nbInputs", static_cast<int>(nbVoltages));
    parametersSet->createParameter("updateStep", step);
    ASSERT_NO_THROW(voltmu->setPARParameters(parametersSet));
    voltmu->addParameters(parameters, false);
    ASSERT_NO_THROW(voltmu->setParametersFromPARFile());
    ASSERT_NO_THROW(voltmu->setSubModelParameters());

    ASSERT_NO_THROW(voltmu->getSize());  // Sets all the sizes
    // Let's check the sizes now so that we never have to do it again. Ever.
    ASSERT_EQ(voltmu->sizeG(), DYN::ModelVoltageMeasurementsUtilities::nbRoots_);
    ASSERT_EQ(voltmu->sizeY(), nbVoltages);
    ASSERT_EQ(voltmu->sizeZ(), nbVoltages + DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_);
    ASSERT_EQ(voltmu->sizeMode(), 0);
    ASSERT_EQ(voltmu->sizeF(), 0);

    // Let's work out the variables and elements.
    std::vector<boost::shared_ptr<Variable> > variables;
    voltmu->defineVariables(variables);
    unsigned int nbVar = ModelVoltageMeasurementsUtilities::nbCalculatedVars_ + ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + 2*nbVoltages;
    ASSERT_EQ(variables.size(), nbVar);
    std::vector<Element> elements;
    std::map<std::string, int> mapElements;
    voltmu->defineElements(elements, mapElements);
    unsigned int baseElem = 2*nbVoltages + DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_ + DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_;
    ASSERT_EQ(elements.size(), 2*baseElem);
    ASSERT_EQ(elements.size(), mapElements.size());
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesDefineMethodsMissingParams) {
    boost::shared_ptr<SubModel> voltmu =
        SubModelFactory::createSubModelFromLib("../DYNModelVoltageMeasurementsUtilities" + std::string(sharedLibraryExtension()));
    voltmu->init(0.);
    std::vector<ParameterModeler> parameters;
    voltmu->defineParameters(parameters);  // stands for VOLTage Measurement Utilities
    ASSERT_EQ(parameters.size(), 2);
    // Define the model parameters
    double step = 12;
    boost::shared_ptr<parameters::ParametersSet> parametersSet = boost::shared_ptr<parameters::ParametersSet>(new parameters::ParametersSet("Parameterset"));
    parametersSet->createParameter("updateStep", step);
    ASSERT_NO_THROW(voltmu->setPARParameters(parametersSet));
    voltmu->addParameters(parameters, false);
    ASSERT_NO_THROW(voltmu->setParametersFromPARFile());
    ASSERT_THROW_DYNAWO(voltmu->setSubModelParameters(), Error::MODELER, KeyError_t::ParameterHasNoValue);
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesSimpleInput) {
    unsigned int nbVoltages = 25;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages);

    unsigned int nbY = nbVoltages;
    std::vector<propertyContinuousVar_t> yTypes(nbY, UNDEFINED_PROPERTY);
    voltmu->setBufferYType(&yTypes[0], 0);
    ASSERT_NO_THROW(voltmu->evalStaticYType());
    ASSERT_EQ(voltmu->sizeY(), nbY);

    ASSERT_EQ(yTypes[0], DYN::EXTERNAL);
    ASSERT_NO_THROW(voltmu->evalStaticFType());  // Does nothing here.
    ASSERT_NO_THROW(voltmu->initializeFromData(boost::shared_ptr<DataInterface>()));

    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    ASSERT_NO_THROW(voltmu->setBufferZ(&z[0], zConnected, 0));
    ASSERT_NO_THROW(voltmu->setBufferY(&voltages[0], nullptr, 0));
    ASSERT_NO_THROW(voltmu->initializeStaticData());
    ASSERT_NO_THROW(voltmu->evalDynamicFType());
    ASSERT_NO_THROW(voltmu->evalDynamicYType());
    // The internal parameters haven't been initialized, it should return 0.
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 0.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), 0.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_), 0.0);
    // We now make sure it is initialized
    ASSERT_NO_THROW(voltmu->getY0());
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 1.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_), nbVoltages * (nbVoltages + 1) / (2*nbVoltages));
    unsigned int nbCalcVars = ModelVoltageMeasurementsUtilities::nbCalculatedVars_;
    ASSERT_THROW_DYNAWO(voltmu->evalCalculatedVarI(nbCalcVars), Error::MODELER, KeyError_t::UndefCalculatedVarI);
    // We're still missing the G buffer. Let's deal with it now.
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);
    ASSERT_NO_THROW(voltmu->setBufferG(&g[0], 0));
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesTimeToUpdate) {
    unsigned int nbVoltages = 25;
    double step = 12;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages, step);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);

    voltmu->evalG(0.0);
    ASSERT_EQ(g[DYN::ModelVoltageMeasurementsUtilities::timeToUpdate_], ROOT_DOWN);
    ASSERT_EQ(voltmu->evalMode(0.0), NO_MODE);

    double next_ts = step + 1.0;
    voltmu->evalG(next_ts);
    ASSERT_EQ(g[DYN::ModelVoltageMeasurementsUtilities::timeToUpdate_], ROOT_UP);
    ASSERT_EQ(voltmu->evalMode(next_ts), ALGEBRAIC_J_UPDATE_MODE);
    // This means we now have a new update. Let's simulate something happening first!
    double minIncreasesBy = 2.0;
    double maxDecreasesBy = 3.0;
    voltages[0] += minIncreasesBy;
    voltages[nbVoltages-1] -=  maxDecreasesBy;
    voltmu->setBufferY(&voltages[0], nullptr, 0);
    ASSERT_NO_THROW(voltmu->evalZ(next_ts));
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 2.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages-1);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_),
                    nbVoltages * (nbVoltages + 1) / (2*nbVoltages) + (minIncreasesBy - maxDecreasesBy)/nbVoltages);
    ASSERT_NO_THROW(voltmu->evalCalculatedVars());
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesConnectionDisconnection) {
    unsigned int nbVoltages = 25;
    double step = 12;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages, step);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);
    // Make sure things are as you expect them to be
    voltmu->evalG(0.0);
    ASSERT_EQ(g[ModelVoltageMeasurementsUtilities::timeToUpdate_], ROOT_DOWN);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 1.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_),
                    nbVoltages * (nbVoltages + 1) / (2*nbVoltages));

    double nextTs = step + 1.0;
    voltmu->evalG(nextTs);
    // Oh! A new calculation is coming. Better make sure we have handled the disconnection we are simulating!
    z[ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = -1.0;
    z[ModelVoltageMeasurementsUtilities::nbDiscreteVars_+nbVoltages-1] = -1;
    voltmu->evalZ(nextTs);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 2.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages-1);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_),
                    nbVoltages * (nbVoltages + 1) / (2*nbVoltages));
    // Simulate another connection before the next required update.
    // Nothing should change.
    double noUpdateTs = nextTs + step/2.;
    voltmu->evalG(noUpdateTs);
    ASSERT_EQ(g[ModelVoltageMeasurementsUtilities::timeToUpdate_], ROOT_DOWN);
    z[ModelVoltageMeasurementsUtilities::nbDiscreteVars_+nbVoltages-1] = 1.0;
    voltmu->evalZ(noUpdateTs);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 2.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages-1);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_),
                    nbVoltages * (nbVoltages + 1) / (2*nbVoltages));
    // If we wait long enough, we'll see the update.
    nextTs *= 2;
    voltmu->evalG(nextTs);
    ASSERT_EQ(g[ModelVoltageMeasurementsUtilities::timeToUpdate_], ROOT_UP);
    voltmu->evalZ(nextTs);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::minValIdx_), 2.0);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::maxValIdx_), nbVoltages);
    ASSERT_EQ(voltmu->evalCalculatedVarI(ModelVoltageMeasurementsUtilities::avgValIdx_),
                    (nbVoltages *(nbVoltages + 1.0)/2.0 - 1.0)/(nbVoltages - 1.0));
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesIndexesOfCalcVar) {
    unsigned int nbVoltages = 25;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);

    // Check the indexes needed for computing the min
    std::vector<int> minIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, minIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(minIndexes[i], i);
    }

    // Check the indexes needed for computing the max
    std::vector<int> maxIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, maxIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(maxIndexes[i], i);
    }

    // Check the indexes needed for computing the average
    std::vector<int> avgIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, avgIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(avgIndexes[i], i);
    }

    ASSERT_THROW_DYNAWO(voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_, avgIndexes),
                    Error::MODELER, KeyError_t::UndefJCalculatedVarI);
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesIndexesOfCalcVarSomeDisconnectedInputs) {
    unsigned int nbVoltages = 25;
    unsigned int nbConnected = 5;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    // Connect only those that are connected (duh!)
    for (std::size_t i=nbConnected; i < nbVoltages; ++i) {
        z[i+ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = -1.0;
    }
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);

    // Check the indexes needed for computing the min
    std::vector<int> minIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, minIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(minIndexes[i], i);
    }

    // Check the indexes needed for computing the max
    std::vector<int> maxIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, maxIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(maxIndexes[i], i);
    }

    // Check the indexes needed for computing the average
    std::vector<int> avgIndexes;
    voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, avgIndexes);

    for (std::size_t i=0; i < nbVoltages; ++i) {
        ASSERT_EQ(avgIndexes[i], i);
    }

    ASSERT_THROW_DYNAWO(voltmu->getIndexesOfVariablesUsedForCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_, avgIndexes),
                    Error::MODELER, KeyError_t::UndefJCalculatedVarI);
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesEvalJCalculatedVars) {
    unsigned int nbVoltages = 25;
    double step = 12;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages, step);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);
    // Make sure things are as you expect them to be
    double t0 = 0.0;
    voltmu->evalG(t0);
    voltmu->evalZ(t0);

    // Check the values of the jacobian for the min
    std::vector<double> gradMinVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, gradMinVal);
    std::vector<double> expectedGradMinVal(voltmu->sizeZ(), 0.);
    expectedGradMinVal[0+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0;

    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMinVal[i], gradMinVal[i]);
    }

    std::vector<double> gradMaxVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, gradMaxVal);
    std::vector<double> expectedGradMaxVal(voltmu->sizeZ(), 0.);
    expectedGradMaxVal[nbVoltages-1+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0;

    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMaxVal[i], gradMaxVal[i]);
    }

    std::vector<double> gradAvgVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, gradAvgVal);
    std::vector<double> expectedGradAvgVal(voltmu->sizeZ(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        expectedGradAvgVal[i+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0/nbVoltages;
    }

    for (std::size_t i = 0; i < nbVoltages; ++i) {
        ASSERT_EQ(expectedGradAvgVal[i], gradAvgVal[i]);
    }

    std::vector<double> wrongIndexGrad(voltmu->sizeZ(), 0.);
    ASSERT_THROW_DYNAWO(voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_, wrongIndexGrad),
                    Error::MODELER, KeyError_t::UndefJCalculatedVarI);
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesEvalJCalculatedVarsWithConnectionUpdates) {
    unsigned int nbVoltages = 25;
    double step = 12;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages, step);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);
    // Make sure things are as you expect them to be
    double t0 = 0.0;
    voltmu->evalG(t0);
    voltmu->evalZ(t0);

    // Check the values of the jacobian for the min
    std::vector<double> gradMinVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, gradMinVal);
    std::vector<double> expectedGradMinVal(voltmu->sizeZ(), 0.);
    expectedGradMinVal[0+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0;

    std::vector<double> gradMaxVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, gradMaxVal);
    std::vector<double> expectedGradMaxVal(voltmu->sizeZ(), 0.);
    expectedGradMaxVal[nbVoltages-1+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0;

    std::vector<double> gradAvgVal(voltmu->sizeZ(), 0.);
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, gradAvgVal);
    std::vector<double> expectedGradAvgVal(voltmu->sizeZ(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        expectedGradAvgVal[i+DYN::ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 1.0/nbVoltages;
    }

    std::vector<double> wrongIndexGrad(voltmu->sizeZ(), 0.);
    ASSERT_THROW_DYNAWO(voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_, wrongIndexGrad),
                    Error::MODELER, KeyError_t::UndefJCalculatedVarI);

    // Add a disconnection before the last update expires: nothing should change.
    double newTs = t0 + step/2.0;  // This is too soon for an update.
    z[ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = -1.0;  // First bus is disconnected.
    voltmu->evalG(newTs);
    voltmu->evalZ(newTs);

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, gradMinVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMinVal[i], gradMinVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, gradMaxVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMaxVal[i], gradMaxVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, gradAvgVal);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        ASSERT_EQ(expectedGradAvgVal[i], gradAvgVal[i]);
    }

    ASSERT_THROW_DYNAWO(voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::nbCalculatedVars_, wrongIndexGrad),
                    Error::MODELER, KeyError_t::UndefJCalculatedVarI);

    // Let's now act on the disconnection!
    newTs += step;
    voltmu->evalG(newTs);
    voltmu->evalZ(newTs);

    // Update the expected grad values.
    // Min has changed, so has the average. Max remains unchanged.
    expectedGradMinVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 0.0;
    expectedGradMinVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + 1] = 1.0;
    expectedGradAvgVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_] = 0.0;
    for (size_t i = ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + 1; i < nbVoltages + ModelVoltageMeasurementsUtilities::nbDiscreteVars_ - 1; ++i) {
        expectedGradAvgVal[i] = 1.0/(nbVoltages - 1);
    }
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, gradMinVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMinVal[i], gradMinVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, gradMaxVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMaxVal[i], gradMaxVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, gradAvgVal);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        ASSERT_EQ(expectedGradAvgVal[i], gradAvgVal[i]);
    }

    // We now make sure the max could change too.
    newTs += step + 1.0;
    z[nbVoltages + ModelVoltageMeasurementsUtilities::nbDiscreteVars_ - 1] = -1.0;

    voltmu->evalG(newTs);
    voltmu->evalZ(newTs);

    // Update the expected grad values.
    // Max has changed, so has the average. Min remains unchanged.
    expectedGradMaxVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + nbVoltages - 1] = 0.0;
    expectedGradMaxVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + nbVoltages - 2] = 1.0;
    expectedGradAvgVal[ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + nbVoltages - 1] = 0.0;
    for (size_t i = ModelVoltageMeasurementsUtilities::nbDiscreteVars_ + 1; i < nbVoltages + ModelVoltageMeasurementsUtilities::nbDiscreteVars_; ++i) {
        expectedGradAvgVal[i] = 1.0/(nbVoltages - 2);
    }
    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::minValIdx_, gradMinVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMinVal[i], gradMinVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::maxValIdx_, gradMaxVal);
    for (std::size_t i = 0; i < voltmu->sizeZ(); ++i) {
        ASSERT_EQ(expectedGradMaxVal[i], gradMaxVal[i]);
    }

    voltmu->evalJCalculatedVarI(DYN::ModelVoltageMeasurementsUtilities::avgValIdx_, gradAvgVal);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        ASSERT_EQ(expectedGradAvgVal[i], gradAvgVal[i]);
    }
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesEvalGNoInit) {
    unsigned int nbVoltages = 25;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    voltmu->setBufferYType(&yTypes[0], 0);
    voltmu->evalStaticYType();
    ASSERT_EQ(voltmu->sizeY(), nbVoltages);

    voltmu->evalStaticFType();  // Does nothing here.
    voltmu->initializeFromData(boost::shared_ptr<DataInterface>());

    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    voltmu->setBufferZ(&z[0], zConnected, 0);
    voltmu->setBufferY(&voltages[0], nullptr, 0);
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);
    voltmu->setBufferG(&g[0], 0);
    voltmu->initializeStaticData();
    voltmu->evalDynamicFType();
    voltmu->evalDynamicYType();

    double t0 = 1.0;  // It could really be anything
    voltmu->evalG(t0);

    // Since things are initialized, the following line shouldn't be an issue!
    ASSERT_NO_THROW(voltmu->getY0());
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesEvalZNoInit) {
    unsigned int nbVoltages = 25;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    voltmu->setBufferYType(&yTypes[0], 0);
    voltmu->evalStaticYType();
    ASSERT_EQ(voltmu->sizeY(), nbVoltages);

    voltmu->evalStaticFType();  // Does nothing here.
    voltmu->initializeFromData(boost::shared_ptr<DataInterface>());

    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    voltmu->setBufferZ(&z[0], zConnected, 0);
    voltmu->setBufferY(&voltages[0], nullptr, 0);
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);
    voltmu->setBufferG(&g[0], 0);
    voltmu->initializeStaticData();
    voltmu->evalDynamicFType();
    voltmu->evalDynamicYType();

    double t0 = 1.0;  // It could really be anything
    voltmu->evalZ(t0);

    // Since things are initialized, the following line shouldn't be an issue!
    ASSERT_NO_THROW(voltmu->getY0());
}

TEST(ModelsVoltageMeasurementUtilities, ModelVoltageMeasurementUtilitiesUselessFunctions) {
    unsigned int nbVoltages = 25;
    double step = 12;
    boost::shared_ptr<SubModel> voltmu = createModelVoltageMeasurementsUtilities(nbVoltages, step);

    std::vector<propertyContinuousVar_t> yTypes(nbVoltages, UNDEFINED_PROPERTY);
    // Connect everything
    std::vector<double> voltages(voltmu->sizeY(), 0.);
    for (std::size_t i = 0; i < nbVoltages; ++i) {
        voltages[i] = static_cast<double>(i + 1);
    }
    std::vector<double> z(voltmu->sizeZ(), 1.);
    bool* zConnected = new bool[voltmu->sizeZ()];
    for (std::size_t i=0; i < voltmu->sizeZ(); ++i) {
        zConnected[i] = true;
    }
    std::vector<state_g> g(voltmu->sizeG(), NO_ROOT);

    setBuffersVMU(voltmu, &yTypes[0], &voltages[0], &z[0], &zConnected[0], &g[0]);

    double t0 = 0.0;
    ASSERT_NO_THROW(voltmu->checkDataCoherence(t0));

    // Harmless coverage
    SparseMatrix mat;

    // Prepare initialization of the model
    ASSERT_NO_THROW(voltmu->setFequations());
    ASSERT_NO_THROW(voltmu->setGequations());
    ASSERT_NO_THROW(voltmu->evalF(t0, UNDEFINED_EQ));
    ASSERT_NO_THROW(voltmu->evalG(0.));
    ASSERT_NO_THROW(voltmu->evalZ(0.));
    ASSERT_NO_THROW(voltmu->evalJt(t0, 0., mat, 0));
    ASSERT_NO_THROW(voltmu->evalJtPrim(t0, 0., mat, 0));

    BitMask* silentZ = new BitMask[voltmu->sizeZ()];
    ASSERT_NO_THROW(voltmu->collectSilentZ(silentZ));

    ASSERT_NO_THROW(voltmu->dumpUserReadableElementList("MyElement"));
}

}  // namespace DYN
