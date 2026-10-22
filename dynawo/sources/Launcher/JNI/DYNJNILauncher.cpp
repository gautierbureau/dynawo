//
// Copyright (c) 2026, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source time domain
// simulation tool for power systems.
//

/**
 * @file  DYNJNILauncher.cpp
 *
 * @brief native methods of com.powsybl.dynawo.jni.DynawoNative: Dynawo run from a JVM, on a network of that JVM
 *
 * The JVM registers its network in the registry of iidm-bridge (IidmBridgeRegistry) under an id. Dynawo reads it there
 * through the JNI backend of iidm-bridge, and writes the final values of its state variables into it: no IIDM file is
 * written or read.
 */
#include <jni.h>

#include <cstdlib>
#include <mutex>
#include <string>

#include <boost/make_shared.hpp>

#include <iidm/NetworkFactory.h>

#include <xml/sax/parser/ParserException.h>

#include "config.h"
#include "gitversion.h"
#include "DYNDataInterfaceIIDM.h"
#include "DYNError.h"
#include "DYNExecUtils.h"
#include "DYNInitXml.h"
#include "DYNIoDico.h"
#include "DYNSignalHandler.h"
#include "DYNSimulationLauncher.h"
#include "DYNTerminate.h"
#include "DYNTrace.h"

namespace {

/// Java class of the exceptions thrown for the failures of Dynawo
const char DYNAWO_NATIVE_EXCEPTION[] = "com/powsybl/dynawo/jni/DynawoNativeException";

/// One simulation at a time: Dynawo keeps state of the process (dictionaries, logs)
std::mutex simulationMutex;

/// Whether the dictionaries of Dynawo are loaded (under simulationMutex)
bool dictionariesLoaded = false;

/**
 * @brief the characters of a Java string
 * @param env JNI environment of the calling thread
 * @param string Java string
 * @return its characters
 */
std::string toString(JNIEnv* env, jstring string) {
  const char* chars = env->GetStringUTFChars(string, nullptr);
  const std::string value(chars);
  env->ReleaseStringUTFChars(string, chars);
  return value;
}

/**
 * @brief throw a Java exception for a failure of Dynawo, a RuntimeException if its class is not found
 * @param env JNI environment of the calling thread
 * @param message message of the exception
 */
void throwDynawoException(JNIEnv* env, const std::string& message) {
  jclass exceptionClass = env->FindClass(DYNAWO_NATIVE_EXCEPTION);
  if (!exceptionClass) {
    env->ExceptionClear();
    exceptionClass = env->FindClass("java/lang/RuntimeException");
  }
  env->ThrowNew(exceptionClass, message.c_str());
}

/**
 * @brief set an environment variable that is not set yet
 * @param name name of the variable
 * @param value its value
 */
void setDefaultEnvVar(const char* name, const std::string& value) {
  if (!std::getenv(name)) {
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 0);
#endif
  }
}

/**
 * @brief the environment that the launcher of an installation of Dynawo (dynawo.sh) gives it, the variables already
 * set being kept
 * @param installDir installation directory of Dynawo
 */
void setDynawoEnvironment(const std::string& installDir) {
  setDefaultEnvVar("DYNAWO_INSTALL_DIR", installDir);
  setDefaultEnvVar("DYNAWO_DDB_DIR", installDir + "/ddb/");
  setDefaultEnvVar("DYNAWO_SCRIPTS_DIR", installDir + "/sbin/");
  setDefaultEnvVar("DYNAWO_RESOURCES_DIR", installDir + "/share/");
  setDefaultEnvVar("DYNAWO_XSD_DIR", installDir + "/share/xsd/");
  setDefaultEnvVar("DYNAWO_DICTIONARIES", "dictionaries_mapping");
  setDefaultEnvVar("DYNAWO_LOCALE", "en_GB");
  setDefaultEnvVar("DYNAWO_USE_XSD_VALIDATION", "false");
}

/**
 * @brief run the job(s) of a jobs file on the network registered in iidm-bridge under an id
 * @param jobsFile jobs file
 * @param networkId id of the network in the registry of iidm-bridge
 */
void runJobs(const std::string& jobsFile, const std::string& networkId) {
  // the JVM handles the signals of the process
  DYN::SignalHandler::disableSignalHandlers();
  DYN::InitXerces xerces;
  DYN::InitLibXml2 libxml2;
  if (!dictionariesLoaded) {
    DYN::IoDicos& dicos = DYN::IoDicos::instance();
    dicos.addPath(getMandatoryEnvVar("DYNAWO_RESOURCES_DIR"));
    dicos.addDicos(getMandatoryEnvVar("DYNAWO_DICTIONARIES"));
    dictionariesLoaded = true;
  }
  // the network of the JVM itself, without any copy
  const boost::shared_ptr<iidm::Network> network = boost::make_shared<iidm::Network>(iidm::NetworkFactory::wrap(networkId));
  launchSimu(jobsFile, DYN::DataInterfaceIIDM::build(network));
}

}  // namespace

extern "C" {

/**
 * @brief version of Dynawo, as printed by dynawo.sh version
 * @param env JNI environment of the calling thread
 * @return version of Dynawo
 */
JNIEXPORT jstring JNICALL
Java_com_powsybl_dynawo_jni_DynawoNative_getVersion(JNIEnv* env, jclass /*clazz*/) {
  const std::string version = std::string(DYNAWO_VERSION_STRING) + " (rev:" + DYNAWO_GIT_BRANCH + "-" + DYNAWO_GIT_HASH + ")";
  return env->NewStringUTF(version.c_str());
}

/**
 * @brief run the job(s) of a jobs file on a network of the JVM; a failure throws a DynawoNativeException
 * @param env JNI environment of the calling thread
 * @param installDir installation directory of Dynawo
 * @param jobsFile jobs file
 * @param networkId id of the network in the registry of iidm-bridge (IidmBridgeRegistry)
 */
JNIEXPORT void JNICALL
Java_com_powsybl_dynawo_jni_DynawoNative_runJobs(JNIEnv* env, jclass /*clazz*/, jstring installDir, jstring jobsFile,
    jstring networkId) {
  std::lock_guard<std::mutex> lock(simulationMutex);
  try {
    setDynawoEnvironment(toString(env, installDir));
    runJobs(toString(env, jobsFile), toString(env, networkId));
    return;
  } catch (const DYN::Error& e) {
    throwDynawoException(env, std::string("DYN Error: ") + e.what());
  } catch (const DYN::MessageError& e) {
    throwDynawoException(env, e.what());
  } catch (const DYN::Terminate& e) {
    throwDynawoException(env, e.what());
  } catch (const xml::sax::parser::ParserException& e) {
    throwDynawoException(env, e.what());
  } catch (const std::exception& e) {
    throwDynawoException(env, e.what());
  } catch (const char* s) {
    throwDynawoException(env, s);
  } catch (const std::string& s) {
    throwDynawoException(env, s);
  } catch (...) {
    throwDynawoException(env, "Unexpected error");
  }
  // a failed job leaves the appenders of its logs
  DYN::Trace::resetCustomAppenders();
}

}  // extern "C"
