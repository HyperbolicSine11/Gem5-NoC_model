/*
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

#include "cpu/testers/garnet_synthetic_traffic/GarnetSyntheticTraffic.hh"

#include <cmath>
#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "debug/GarnetSyntheticTraffic.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

int TESTER_NETWORK=0;

bool
GarnetSyntheticTraffic::CpuPort::recvTimingResp(PacketPtr pkt)
{
    tester->completeRequest(pkt);
    return true;
}

void
GarnetSyntheticTraffic::CpuPort::recvReqRetry()
{
    tester->doRetry();
}

void
GarnetSyntheticTraffic::sendPkt(PacketPtr pkt)
{
    if (!cachePort.sendTimingReq(pkt)) {
        retryPkt = pkt; // RubyPort will retry sending
    }
    numPacketsSent++;
}

GarnetSyntheticTraffic::GarnetSyntheticTraffic(const Params &p)
    : ClockedObject(p),
      tickEvent([this]{ tick(); }, "GarnetSyntheticTraffic tick",
                false, Event::CPU_Tick_Pri),
      cachePort("GarnetSyntheticTraffic", this),
      retryPkt(NULL),
      size(p.memory_size),
      blockSizeBits(p.block_offset),
      numDestinations(p.num_dest),
      simCycles(p.sim_cycles),
      numPacketsMax(p.num_packets_max),
      numPacketsSent(0),
      singleSender(p.single_sender),
      singleDest(p.single_dest),
      trafficType(p.traffic_type),
      injRate(p.inj_rate),
      injVnet(p.inj_vnet),
      precision(p.precision),
      responseLimit(p.response_limit),
      requestorId(p.system->getRequestorId(this))
{
    // set up counters
    noResponseCycles = 0;
    schedule(tickEvent, 0);

    initTrafficType();
    if (trafficStringToEnum.count(trafficType) == 0) {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }
    traffic = trafficStringToEnum[trafficType];

    id = TESTER_NETWORK++;
    DPRINTF(GarnetSyntheticTraffic,"Config Created: Name = %s , and id = %d\n",
            name(), id);
}

Port &
GarnetSyntheticTraffic::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "test")
        return cachePort;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
GarnetSyntheticTraffic::init()
{
    numPacketsSent = 0;
}


void
GarnetSyntheticTraffic::completeRequest(PacketPtr pkt)
{
    DPRINTF(GarnetSyntheticTraffic,
            "Completed injection of %s packet for address %x\n",
            pkt->isWrite() ? "write" : "read\n",
            pkt->req->getPaddr());

    assert(pkt->isResponse());
    noResponseCycles = 0;
    delete pkt;
}


void
GarnetSyntheticTraffic::tick()
{
    if (++noResponseCycles >= responseLimit) {
        fatal("%s deadlocked at cycle %d\n", name(), curTick());
    }

    // make new request based on injection rate
    // (injection rate's range depends on precision)
    // - generate a random number between 0 and 10^precision
    // - send pkt if this number is < injRate*(10^precision)
    bool sendAllowedThisCycle;
    double injRange = pow((double) 10, (double) precision);
    unsigned trySending = random_mt.random<unsigned>(0, (int) injRange);
    if (trySending < injRate*injRange)
        sendAllowedThisCycle = true;
    else
        sendAllowedThisCycle = false;

    // always generatePkt unless fixedPkts or singleSender is enabled
    if (sendAllowedThisCycle) {
        bool senderEnable = true;

        if (numPacketsMax >= 0 && numPacketsSent >= numPacketsMax)
            senderEnable = false;

        if (singleSender >= 0 && id != singleSender)
            senderEnable = false;

        if (senderEnable)
            generatePkt();
    }

    // Schedule wakeup
    if (curTick() >= simCycles)
        exitSimLoop("Network Tester completed simCycles");
    else {
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
GarnetSyntheticTraffic::generatePkt()
{
    int num_destinations = numDestinations;
    int radix = (int) sqrt(num_destinations);
    unsigned destination = id;
    // unsigned destination1 = destination+1;
    // unsigned destination2 = destination+2;
    // unsigned destination3 = destination+3;
    // unsigned destination4 = destination+4;
    // unsigned destination5 = destination+5;
    // unsigned destination6 = destination+6;
    // unsigned destination7 = destination+7;


    int dest_x = -1;
    int dest_y = -1;
    int source = id;
    // int source1 = id+1;
    // int source2 = id+2;
    // int source3 = id+3;
    // int source4 = id+4;
    // int source5 = id+5;
    // int source6 = id+6;
    // int source7 = id+7;

    int src_x = id%radix;
    int src_y = id/radix;
    // int src_x1 = (id+1)%radix;
    // int src_y1 = (id+1)/radix;
    // int src_x2 = (id+2)%radix;
    // int src_y2 = (id+2)/radix;
    // int src_x3 = (id+3)%radix;
    // int src_y3 = (id+3)/radix;
    // int src_x4 = (id+4)%radix;
    // int src_y4 = (id+4)/radix;
    // int src_x5 = (id+5)%radix;
    // int src_y5 = (id+5)/radix;
    // int src_x6 = (id+6)%radix;
    // int src_y6 = (id+6)/radix;
    // int src_x7 = (id+7)%radix;
    // int src_y7 = (id+7)/radix;

    if (singleDest >= 0)
    {
        destination = singleDest;
    } else if (traffic == UNIFORM_RANDOM_) {
        destination = random_mt.random<unsigned>(0, num_destinations - 1);
    } else if (traffic == BIT_COMPLEMENT_) {
        dest_x = radix - src_x - 1;
        dest_y = radix - src_y - 1;
        destination = dest_y*radix + dest_x;
    } else if (traffic == BIT_REVERSE_) {
        unsigned int straight = source;
        unsigned int reverse = source & 1; // LSB

        int num_bits = (int) log2(num_destinations);

        for (int i = 1; i < num_bits; i++)
        {
            reverse <<= 1;
            straight >>= 1;
            reverse |= (straight & 1); // LSB
        }
        destination = reverse;
    } else if (traffic == BIT_ROTATION_) {
        if (source%2 == 0)
            destination = source/2;
        else // (source%2 == 1)
            destination = ((source/2) + (num_destinations/2));
    } else if (traffic == NEIGHBOR_) {
            dest_x = (src_x + 1) % radix;
            dest_y = src_y;
            destination = dest_y*radix + dest_x;
    } else if (traffic == SHUFFLE_) {
        if (source < num_destinations/2)
            destination = source*2;
        else
            destination = (source*2 - num_destinations + 1);
    } else if (traffic == TRANSPOSE_) {
            dest_x = src_y;
            dest_y = src_x;
            destination = dest_y*radix + dest_x;
    } else if (traffic == TORNADO_) {
        dest_x = (src_x + (int) ceil(radix/2) - 1) % radix;
        dest_y = src_y;
        destination = dest_y*radix + dest_x;
    } else if (traffic == HOT_SPOT_) {
        destination = random_mt.random<unsigned>(
            (int)(num_destinations/2)-1 , (int)(num_destinations/2)
            );
    } else if (traffic == CUSTOM_) {
        int center_axis = (int)(radix/2);
        int center = center_axis*radix + center_axis;
        if(source == center)
            destination = center_axis*radix + center_axis - 1;
        else if(source == center - 1)
            destination = center;
        else
            destination = random_mt.random<unsigned>(0, num_destinations - 1);
    }
    else {
        fatal("Unknown Traffic Type: %s!\n", traffic);
    }

    // The source of the packets is a cache.
    // The destination of the packets is a directory.
    // The destination bits are embedded in the address after byte-offset.
    Addr paddr =  destination;
    paddr <<= blockSizeBits;
    // Addr paddr1 =  destination1;
    // paddr1 <<= blockSizeBits;
    // Addr paddr2 =  destination2;
    // paddr2 <<= blockSizeBits;
    // Addr paddr3 =  destination3;
    // paddr3 <<= blockSizeBits;
    // Addr paddr4 =  destination4;
    // paddr4 <<= blockSizeBits;
    // Addr paddr5 =  destination5;
    // paddr5 <<= blockSizeBits;
    // Addr paddr6 =  destination6;
    // paddr6 <<= blockSizeBits;
    // Addr paddr7 =  destination7;
    // paddr7 <<= blockSizeBits;
    unsigned access_size = 1; // Does not affect Ruby simulation

    // Modeling different coherence msg types over different msg classes.
    //
    // GarnetSyntheticTraffic assumes the Garnet_standalone coherence protocol
    // which models three message classes/virtual networks.
    // These are: request, forward, response.
    // requests and forwards are "control" packets (typically 8 bytes),
    // while responses are "data" packets (typically 72 bytes).
    //
    // Life of a packet from the tester into the network:
    // (1) This function generatePkt() generates packets of one of the
    //     following 3 types (randomly) : ReadReq, INST_FETCH, WriteReq
    // (2) mem/ruby/system/RubyPort.cc converts these to RubyRequestType_LD,
    //     RubyRequestType_IFETCH, RubyRequestType_ST respectively
    // (3) mem/ruby/system/Sequencer.cc sends these to the cache controllers
    //     in the coherence protocol.
    // (4) Network_test-cache.sm tags RubyRequestType:LD,
    //     RubyRequestType:IFETCH and RubyRequestType:ST as
    //     Request, Forward, and Response events respectively;
    //     and injects them into virtual networks 0, 1 and 2 respectively.
    //     It immediately calls back the sequencer.
    // (5) The packet traverses the network (simple/garnet) and reaches its
    //     destination (Directory), and network stats are updated.
    // (6) Network_test-dir.sm simply drops the packet.
    //
    MemCmd::Command requestType;

    RequestPtr req = nullptr;
    // RequestPtr req1 = nullptr;
    // RequestPtr req2 = nullptr;
    // RequestPtr req3 = nullptr;
    // RequestPtr req4 = nullptr;
    // RequestPtr req5 = nullptr;
    // RequestPtr req6 = nullptr;
    // RequestPtr req7 = nullptr;

    Request::Flags flags;

    // Inject in specific Vnet
    // Vnet 0 and 1 are for control packets (1-flit)
    // Vnet 2 is for data packets (5-flit)
    int injReqType = injVnet;

    if (injReqType < 0 || injReqType > 2)
    {
        // randomly inject in any vnet
        injReqType = random_mt.random(0, 2);
    }

    if (injReqType == 0) {
        // generate packet for virtual network 0
        requestType = MemCmd::ReadReq;
        req = std::make_shared<Request>(paddr, access_size, flags, requestorId);
        // req1 = std::make_shared<Request>(paddr1, access_size, flags, requestorId);
        // req2 = std::make_shared<Request>(paddr2, access_size, flags, requestorId);
        // req3 = std::make_shared<Request>(paddr3, access_size, flags, requestorId);
        // req4 = std::make_shared<Request>(paddr4, access_size, flags, requestorId);
        // req5 = std::make_shared<Request>(paddr5, access_size, flags, requestorId);
        // req6 = std::make_shared<Request>(paddr6, access_size, flags, requestorId);
        // req7 = std::make_shared<Request>(paddr7, access_size, flags, requestorId);

    } else if (injReqType == 1) {
        // generate packet for virtual network 1
        requestType = MemCmd::ReadReq;
        flags.set(Request::INST_FETCH);
        req = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        req->setPaddr(paddr);
        // req1 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req1->setPaddr(paddr1);
        // req2 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req2->setPaddr(paddr2);
        // req3 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req3->setPaddr(paddr3);
        // req4 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req4->setPaddr(paddr4);
        // req5 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req5->setPaddr(paddr5);
        // req6 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req6->setPaddr(paddr6);
        // req7 = std::make_shared<Request>(0x0, access_size, flags, requestorId, 0x0, 0);
        // req7->setPaddr(paddr7);
    } else {  // if (injReqType == 2)
        // generate packet for virtual network 2
        requestType = MemCmd::WriteReq;
        req = std::make_shared<Request>(paddr, access_size, flags, requestorId);
        // req1 = std::make_shared<Request>(paddr1, access_size, flags, requestorId);
        // req2 = std::make_shared<Request>(paddr2, access_size, flags, requestorId);
        // req3 = std::make_shared<Request>(paddr3, access_size, flags, requestorId);
        // req4 = std::make_shared<Request>(paddr4, access_size, flags, requestorId);
        // req5 = std::make_shared<Request>(paddr5, access_size, flags, requestorId);
        // req6 = std::make_shared<Request>(paddr6, access_size, flags, requestorId);
        // req7 = std::make_shared<Request>(paddr7, access_size, flags, requestorId);

    }

    req->setContext(id);
    // req1->setContext(id+1);
    // req2->setContext(id+2);
    // req3->setContext(id+3);
    // req4->setContext(id+4);
    // req5->setContext(id+5);
    // req6->setContext(id+6);
    // req7->setContext(id+7);

    //No need to do functional simulation
    //We just do timing simulation of the network

    DPRINTF(GarnetSyntheticTraffic,
            "Generated packet with destination %d, embedded in address %x\n",
            destination, req->getPaddr());

    PacketPtr pkt = new Packet(req, requestType);
    pkt->dataDynamic(new uint8_t[req->getSize()]);
    pkt->senderState = NULL;

    sendPkt(pkt);
}

void
GarnetSyntheticTraffic::initTrafficType()
{
    trafficStringToEnum["bit_complement"] = BIT_COMPLEMENT_;
    trafficStringToEnum["bit_reverse"] = BIT_REVERSE_;
    trafficStringToEnum["bit_rotation"] = BIT_ROTATION_;
    trafficStringToEnum["neighbor"] = NEIGHBOR_;
    trafficStringToEnum["shuffle"] = SHUFFLE_;
    trafficStringToEnum["tornado"] = TORNADO_;
    trafficStringToEnum["transpose"] = TRANSPOSE_;
    trafficStringToEnum["uniform_random"] = UNIFORM_RANDOM_;
    trafficStringToEnum["hot_spot"] = HOT_SPOT_;
    trafficStringToEnum["custom"] = CUSTOM_;

}

void
GarnetSyntheticTraffic::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        retryPkt = NULL;
    }
}

void
GarnetSyntheticTraffic::printAddr(Addr a)
{
    cachePort.printAddr(a);
}

} // namespace gem5
