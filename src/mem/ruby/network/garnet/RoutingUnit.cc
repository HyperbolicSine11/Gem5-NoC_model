/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "mem/ruby/network/garnet/RoutingUnit.hh"

#include "base/cast.hh"
#include "base/compiler.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/OutputUnit.hh"
#include "mem/ruby/network/garnet/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include <random>

namespace gem5
{

namespace ruby
{

namespace garnet
{

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();

}

void
RoutingUnit::addRoute(std::vector<NetDest>& routing_table_entry)
{
    if (routing_table_entry.size() > m_routing_table.size()) {
        m_routing_table.resize(routing_table_entry.size());
    }
    for (int v = 0; v < routing_table_entry.size(); v++) {
        m_routing_table[v].push_back(routing_table_entry[v]);
    }
}

void
RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

bool
RoutingUnit::supportsVnet(int vnet, std::vector<int> sVnets)
{
    // If all vnets are supported, return true
    if (sVnets.size() == 0) {
        return true;
    }

    // Find the vnet in the vector, return true
    if (std::find(sVnets.begin(), sVnets.end(), vnet) != sVnets.end()) {
        return true;
    }

    // Not supported vnet
    return false;
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */
int
RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

        if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(
            m_routing_table[vnet][link])) {

            if (m_weight_table[link] == min_weight) {
                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0) {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}


void
RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx]  = inport_dirn;
}

void
RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx]  = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

int
RoutingUnit::outportCompute(RouteInfo route, int inport,
                            PortDirection inport_dirn)
{
    int outport = -1;

    if (route.dest_router == m_router->get_id()) {

        // Multiple NIs may be connected to this router,
        // all with output port direction = "Local"
        // Get exact outport id from table
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        return outport;
    }

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
        (RoutingAlgorithm) m_router->get_net_ptr()->getRoutingAlgorithm();

    switch (routing_algorithm) {
        case TABLE_:  outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
        case XY_:     outport =
            outportComputeXY(route, inport, inport_dirn); break;
        // any custom algorithm
        case CUSTOM_: outport =
            outportComputeCustom(route, inport_dirn, m_router->getQTable(), m_router->getCongestionTable()); break;
        default: outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
    }

    assert(outport != -1);
    return outport;
}

// XY routing implemented using port directions
// Only for reference purpose in a Mesh
// By default Garnet uses the routing table
int
RoutingUnit::outportComputeXY(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0) {
        if (x_dirn) {
            assert(inport_dirn == "Local" || inport_dirn == "West");
            outport_dirn = "East";
        } else {
            assert(inport_dirn == "Local" || inport_dirn == "East");
            outport_dirn = "West";
        }
    } else if (y_hops > 0) {
        if (y_dirn) {
            // "Local" or "South" or "West" or "East"
            assert(inport_dirn != "North");
            outport_dirn = "North";
        } else {
            // "Local" or "North" or "West" or "East"
            assert(inport_dirn != "South");
            outport_dirn = "South";
        }
    } else {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        panic("x_hops == y_hops == 0");
    }

    return m_outports_dirn2idx[outport_dirn];
}

// SHX
// QRoutingAlgorithm
int
RoutingUnit::outportComputeCustom(RouteInfo route,
                                  PortDirection inport_dirn,
                                  std::unordered_map<PortDirection, float>* q_table,
                                  std::unordered_map<PortDirection, uint32_t*> congestion_table)
{

    PortDirection relative_dir;
    PortDirection oppsite_relative_dir;
    PortDirection outport_dirn = "Unknown";
    PortDirection least_congested_dir = "Unkown";
    std::set<PortDirection> masked_direction;
    std::set<PortDirection> dyxy_candidate_dir;
    bool use_QRA = false;
    PortDirection q_table_dir = "Unknown";

    int vnet = route.vnet;
    NetDest msg_destination = route.net_dest;


    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);//indicates dst is on x axis
    int y_hops = abs(dest_y - my_y);//indicates dst is on y axis

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > y_hops) {
        if (x_dirn) { relative_dir = "East";}
        else { relative_dir = "West";}
    } else if (x_hops < y_hops) {
        if (y_dirn) { relative_dir = "North";}
        else { relative_dir = "South";}
    } else {//x_hops == y_hops
        if (x_dirn > 0 && y_dirn >0) {
            if (inport_dirn == "East") relative_dir = "North";//Prevents turn-around
            else if (inport_dirn == "North") relative_dir = "East";
            else relative_dir = "North";
        }
        else if (x_dirn > 0 && y_dirn <0) {
            if (inport_dirn == "East") relative_dir = "South";
            else if (inport_dirn == "South") relative_dir = "East";
            else relative_dir = "South";
        }
        else if (x_dirn < 0 && y_dirn <0) {
            if (inport_dirn == "West") relative_dir = "South";
            else if (inport_dirn == "South") relative_dir = "West";
            else relative_dir = "South";
        }
        else if (x_dirn < 0 && y_dirn >0) {
            if (inport_dirn == "West") relative_dir = "North";
            else if (inport_dirn == "North") relative_dir = "West";
            else relative_dir = "North";
        }
    }

    if (relative_dir == "North") oppsite_relative_dir = "South";
    else if (relative_dir == "East" ) oppsite_relative_dir = "West";
    else if (relative_dir == "South") oppsite_relative_dir = "North";
    else oppsite_relative_dir = "East";

    //Get masked direction: oppsite_relative_dir and inport_dirn
    masked_direction.insert(inport_dirn);
    masked_direction.insert(oppsite_relative_dir);

    std::pair(q_table_dir, use_QRA) = q_table_lookup(masked_direction, q_table);


    for (int link = 0; link < m_routing_table[vnet].size(); link++) {
        PortDirection link_to_action = "Unknown";
        if (msg_destination.intersectionIsNotEmpty(m_routing_table[vnet][link])) {
            link_to_action = m_inports_idx2dirn[link];
            dyxy_candidate_dir.insert(link_to_action);
        }

    }

    least_congested_dir = find_least_congested_dir(dyxy_candidate_dir,congestion_table);

    if (use_QRA) outport_dirn = q_table_dir;
    else outport_dirn = least_congested_dir;

    return m_outports_dirn2idx[outport_dirn];

}

//SHX
void
RoutingUnit::calQTable(int outport, Router *router, int outvc)
{
    float alpha = 0.2;
    float gamma = 0.5;
    int r_value;
    float dest_router_max_q = 0;
    PortDirection port_2_dirn = m_outports_idx2dirn[outport];
    std::unordered_map<PortDirection, float>* q_table = router->getQTable();
    std::unordered_map<PortDirection, float*> q_max_table = router->getQmaxTable();
    OutputUnit *output_unit = router->getOutputUnit((unsigned)outport);
    int credit_count = output_unit->get_credit_count(outvc);
    if(credit_count == 0){
        r_value = 0;
        DPRINTF(RubyNetwork, "Has no free credit");}
    else if(credit_count <= 2)
        r_value = 1;
    else
        r_value = 2;
    // std::cout<<"From "<<router->get_id()<<" to "<<port_2_dirn<<" r_value is "<<r_value<<std::endl;
    if(port_2_dirn != "Local") {
        dest_router_max_q = *q_max_table[port_2_dirn];
        for (auto& pair : *q_table) {
            if(port_2_dirn == pair.first) {
                pair.second = (1-alpha) * pair.second + alpha * (r_value + gamma * dest_router_max_q);
            }
        }
    }
}


//SHX
bool contains(const std::set<PortDirection>& list, PortDirection dir) {
    return std::find(list.begin(), list.end(), dir) != list.end();
}

//SHX
std::pair<PortDirection, bool> q_table_lookup(std::set<PortDirection> masked_direction,
                                              std::unordered_map<PortDirection, float>* q_table) {
    float threshold = 1;//This is the value to trigger QRA, lesser difference will not kick in
    float abs_difference = 0;
    float max_abs_difference = 0;
    bool larger = false;
    float max_q = 0;
    bool use_QRA = false;
    bool use_best_action = true;
    uint32_t sum_actions = 0;
    PortDirection max_q_dir = "Unknown";
    PortDirection output_dir = "Unknown";
    std::vector<PortDirection> candidate_dir;
    for (auto& pair : *q_table){
        PortDirection direction = pair.first;
        float q_value = pair.second;
        if(!contains(masked_direction, direction)) {
            candidate_dir.push_back(direction);
            sum_actions += 1;
            abs_difference = abs(pair.second - max_q);
            larger = (q_value >= max_q);
            if (larger) {
                max_q = q_value;
                max_q_dir = direction;
            }
            if (abs_difference > max_abs_difference) max_abs_difference = abs_difference;
        }
    }
    if (max_abs_difference > threshold) use_QRA = true;
    use_best_action = use_qmax_action(sum_actions);
    if (candidate_dir.size()==1) output_dir = max_q_dir;
    else if (use_best_action) output_dir = max_q_dir;
    else {
        candidate_dir.erase(std::remove(candidate_dir.begin(), candidate_dir.end(), max_q_dir), candidate_dir.end());
        output_dir = candidate_dir[rand() % candidate_dir.size()];
    }
    return std::make_pair(output_dir, use_QRA);
}

bool use_qmax_action(u_int32_t sum_actions) {
    float epsilon = 0.1;
    float best_action_possibility = 0;
    bool use_best_action = false;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    float random_value = distribution(generator);
    best_action_possibility = 1 - epsilon + (epsilon / sum_actions);
    if (random_value <= best_action_possibility) { use_best_action = true; }
    return use_best_action;
}

PortDirection find_least_congested_dir(std::set<PortDirection> candidate_dir,
                                       std::unordered_map<PortDirection, uint32_t*> m_congestion_table){
    PortDirection min_dir = "Unknown"; // 用于存储最小值的 PortDirection
    uint32_t min_value = std::numeric_limits<uint32_t>::max(); // 初始化为 uint32_t 可能的最大值

    for (const auto& pair : m_congestion_table) {
        PortDirection direction = pair.first;
        if (contains(candidate_dir, direction)){
            // 检查指针是否非空
            if (pair.second != nullptr && *(pair.second) < min_value) {
                min_value = *(pair.second);
                min_dir = pair.first;
            }
        }
    }
    return min_dir;
}

// // Marked
// std::vector<int>
// RoutingUnit::get_weight_table()
// {
//     return m_weight_table;
// }

} // namespace garnet
} // namespace ruby
} // namespace gem5
