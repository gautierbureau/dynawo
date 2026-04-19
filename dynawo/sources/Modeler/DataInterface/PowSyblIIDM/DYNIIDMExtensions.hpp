//
// Copyright (c) 2021, RTE (http://www.rte-france.com)
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
 * @file DYNIIDMExtensions.hpp
 * @brief File for external IIDM extensions management
 */

#ifndef MODELER_DATAINTERFACE_POWSYBLIIDM_DYNIIDMEXTENSIONS_HPP_
#define MODELER_DATAINTERFACE_POWSYBLIIDM_DYNIIDMEXTENSIONS_HPP_

#include "DYNIIDMExtensionsTraits.hpp"

#include <boost/filesystem.hpp>
#include <functional>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace DYN {

/// @brief IIDM extensions management wrapper
class IIDMExtensions {
 public:
  /// @brief Alias type for base extension create function
  template<class T>
  using CreateFunctionBase = T*(typename IIDMExtensionTrait<T>::NetworkComponentType&);

  /// @brief Alias type for extension create function with STL wrapper
  template<class T>
  using CreateFunction = std::function<CreateFunctionBase<T> >;

  /// @brief Alias type for extension destruction function
  template<class T>
  using DestroyFunctionBase = void(T*);
  /// @brief Alias type for extension destruction function with STL wrapper
  template<class T>
  using DestroyFunction = std::function<DestroyFunctionBase<T> >;

  /// @brief Alias type for extension definition (creation/destruction)
  template<class T>
  using ExtensionDefinition = std::tuple<CreateFunction<T>, DestroyFunction<T> >;

  /// @brief Extension enum to make access to Extension Definition fields easier
  enum ExtensionDefinitionIndex {
    CREATE_FUNCTION = 0,  ///< Extension creation function
    DESTROY_FUNCTION      ///< Extension destruction function
  };

  /**
   * @brief Find library path from DYNAMO environment
   * @returns the IIDM extension library path
   */
  static boost::filesystem::path findLibraryPath();

 public:
  /**
   * @brief Retrieve the extension definition
   *
   * iidm-bridge migration: the dlopen-based plugin loader is currently disabled
   * because the new iidm-bridge backend does not expose the necessary type
   * surface (no generic Connectable base, no ExtensionProviders registry).
   * Until the design is reworked on the Dynawo side, every call returns the
   * default no-op create/destroy pair so production code paths remain correct
   * (custom extensions simply look "not present").
   *
   * @param libPath the path of the library containing the extension (ignored)
   * @returns the extension definition
   */
  template<class T>
  static ExtensionDefinition<T> getExtension(const std::string& /*libPath*/) {
    return buildDefaultExtensionDefinition<T>();
  }

 private:
  /**
   * @brief Build a default extension definition
   *
   * By default, the create function returns a NULL pointer
   * By default the destroy function does nothing
   *
   * @returns an extension definition that does nothing
   */
  template<class T>
  static inline ExtensionDefinition<T> buildDefaultExtensionDefinition() {
    auto defaultCreate = [](typename IIDMExtensionTrait<T>::NetworkComponentType&) -> T* { return nullptr; };
    auto defaultDestroy = [](T*) {
      // do nothing
    };
    return ExtensionDefinition<T>(defaultCreate, defaultDestroy);
  }
};
}  // namespace DYN

#endif  // MODELER_DATAINTERFACE_POWSYBLIIDM_DYNIIDMEXTENSIONS_HPP_
