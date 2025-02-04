/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#ifndef __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_HPP__
#define __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_HPP__

#include "neighbor_generator.hpp"
#include "capabilities_interface.hpp"
#include "vehicle_state.hpp"

namespace arbitrator
{
    /**
     * \brief Implementation of the NeighborGenerator interface using plugins
     * 
     * Queries plugins via the CapabilitiesInterface to contribute additional
     * maneuvers as the potential child/neighbor nodes of a plan in progress.
     * 
     * \tparam T The type of CapabilitiesInterface to use. Templated to enable 
     *      testing
     */
    template <class T>
    class PluginNeighborGenerator : public NeighborGenerator
    {
        public:
            /**
             * Constructor for PluginNeighborGenerator
             * \param ci A capabilties interface for accessing the plugins
             */
            PluginNeighborGenerator(std::shared_ptr<T> ci) :
                ci_(ci) {};

            /**
             * Generates a list of neighbor states for the given plan using 
             * the plugins available to the system
             * \param plan The plan that is the current search state
             * \param initial_state The initial state of the vehicle at the start of plan. This will be provided to planners for specific use when plan is empty
             * \return A list of subsequent plans building on top of the input plan
             */
            std::vector<carma_planning_msgs::msg::ManeuverPlan> generate_neighbors(carma_planning_msgs::msg::ManeuverPlan plan, const VehicleState& initial_state) const;
        private:
            std::shared_ptr<T> ci_;
    };
};

#include "plugin_neighbor_generator.tpp"

#endif //__ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_HPP__