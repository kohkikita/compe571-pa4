PA4 – Virtual Memory Simulation and Page Replacement Algorithms

vm_sim.py
Main program for PA4.
Implements a virtual memory simulator with per-process page tables and a fixed-size physical memory.

Features:
	- Virtual to physical address translation (16-bit addresses, 512-byte pages)
	- Per-process page tables (128 entries per process)
	- Physical memory with 32 frames
	- Page fault, disk access, and dirty write-back accounting
	- Page replacement policy:
		- RAND (Random)

Input Files:
data1.txt – Memory reference trace 1
data2.txt – Memory reference trace 2

Each line contains: process ID, virtual address, and access type (R or W).


Execution Examples:
python vm_sim.py data1.txt
python vm_sim.py data2.txt

The replacement policy is selected in the main() function of vm_sim.py.

Output:
Printed to the console:
	- Page faults
	- Disk accesses
	- Dirty page writes

Notes:
	- RAND supports an optional random seed for reproducible results.