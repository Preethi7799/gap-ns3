# GAP: A Gas-Aware Priority Protocol for Indoor CO2 Monitoring Using NS-3

This repository contains the NS-3 simulation codes for the GAP protocol paper.

## Description

GAP is a gas-aware priority forwarding protocol for indoor CO2 monitoring networks. It classifies CO2 readings into normal, high, and critical priority levels and uses priority, link quality, progress toward the gateway, energy factor, and queue load for forwarding decisions.

## NS-3 Version

The simulations were tested using:

- NS-3 version: ns-3.43
- Platform: Ubuntu/WSL
- Wireless standard: IEEE 802.11g
- Topology: 15-node indoor grid
- Gateway: Node 0
- Simulation time: 40 seconds per run
- Number of runs: 10, 20

## Files

- `scratch/gap-protocol-bf-delay.cc`: Improved GAP implementation with reduced critical packet delay.
- `scratch/gap-protocol.cc`: Modular GAP implementation.
- `scratch/gap-protocol.h`: GAP class declarations.
- `scratch/gap-files/gap-header.inc`: GAP packet header implementation.
- `scratch/gap-files/gap-helper.inc`: Optional GAP helper implementation.
- `scratch/aodv-base.cc`: AODV baseline simulation.
- `scratch/olsr-base.cc`: OLSR baseline simulation.

## How to Run

Copy the files into the `scratch/` directory of NS-3.43.

Example:

```bash
cd ~/ns3-work/ns-allinone-3.43/ns-3.43

./ns3 run "scratch/gap-protocol-bf-delay --RngRun=1"
./ns3 run "scratch/aodv-base --RngRun=1"
./ns3 run "scratch/olsr-base --RngRun=1"

## RngRun number could be increased to check the performance of protocols.

## To run 20 independent runs

for i in {1..20}
do
  ./ns3 run "scratch/gap-protocol-bf-delay --RngRun=$i" > gap_run_$i.txt 2>/dev/null
done

## If you use this code, please cite:

GAP: A Gas-Aware Priority Protocol for Indoor CO2 Monitoring Using NS-3.
