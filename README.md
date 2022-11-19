# MATH3205-Project-VRT-with-Drones

MATH3205 Project
Haolong Du (46272784) & Oscar Davine (45285714)

This document is meant to provide a guide on how to use the code 

## 1 Structure:

46272784_45285714_MATH3205_Project:\
-- Benders \
---- BendersTestLog (A directory that stores plots and logs) \
---- BendersV2.py (Code for the LBBD approch) \

-- MIP \
---- MIPLazyTestLog (Another directory that stores plots and logs) \
---- MIPTestLog (AAnother directory that stores plots and logs) \
---- MIP.py (Code for the MIP approch) \
---- MIPlazy.py (Code for the MIP with lazy GCS constraints) \

-- test \
---- testCases (A directory that contains all test sets) \
---- load_test.py (Dataloader) \
---- test.py (Code for batch testing) \

-- Papers&PPT \
---- An_Exact_Algorithm_for_Heterogeneous_Drone-TruckRouting_Problem.pdf \
---- MT1.pdf (The PowerPoint on the above paper.) \

-- 46272784_45285714_MATH3205_FINAL_REPORT.pdf (Our final report) \

-- README.md (You are reading it) \

## 2 Instruction:
Please DO NOT add / remove any file / directory.
For running a single algorithm:
	1. Open the file containing the algorithm (MIP.py or MIPlazy.py or BendersV2.py) AND and dataloader (load_test.py)
	2. Change the variable CWD in main() in BOTH FILES to r'your/directory/46272784_45285714_MATH3205_Project'
	3. Find which test you would like to run, then enter them in the main function
	4. Run the algorithm file, and the resulting plot and log will be in the {}TestLog directory
For batch testing:
	1. Open the dataloader (load_test.py) AND the test file (test.py)
	2. Change the variable CWD in main() in BOTH FILES to r'your/directory/46272784_45285714_MATH3205_Project'
	3. Choose one or more sets to run in test.py
	4. Run test.py, the result will be recorded in /test/testResult.csv. Plot and log will still be saved in {}TestLog directory

Note: DO NOT run test.py with testResult.csv open
