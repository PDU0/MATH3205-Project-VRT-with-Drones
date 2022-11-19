import time
import csv
import sys
############ Please Change ###############################################
CWD = "YOUR/WORKING/DIR"
##########################################################################

sys.path.insert(1, CWD+r'\test')
from load_test import load_test
sys.path.insert(1, CWD+r'\MIP')
from MIP import mip
from MIPlazy import mipLazy
sys.path.insert(1, CWD+r'\Benders')
from BendersV2 import BendersV2


def runTest(testNames, timeLimit, which='all'):
    global CWD
    TestDir = CWD + r'\test\testCases'
    
    if which == 'all':
        # Write a header in the csv file
        detailCSVName = CWD + r'\test\testResult.csv'
        detailCSV = open(detailCSVName, 'w+', newline='')
        writer = csv.writer(detailCSV)
        writer.writerow(['Name', 'MIP Time', 'MIP objVal', 
                         'MIPLazy Time', 'MIPLazy objVal', '# GCS Cuts', 
                         'Primal Heuristic', 'Truck Route', 'WLow', 'Benders', 
                         'Benders Time', 'Benders objVal', 
                         '# GCS Cuts', '# Feas Cut', '# Opti Cut'])
        detailCSV.close()
        
        allTest = {}
        for name in testNames:
            tests = load_test(name, TestDir)
            allTest.update(tests)
            for i in allTest:
                row = [i]
                testData = allTest[i]
                # MIP
                t0 = time.time()
                mipObj = mip(testData, i, timeLimit, CWD)
                t1 = time.time()
                row.append(round(t1-t0, 4))
                row.append(round(mipObj, 2))
                
                # MIPLazy
                t0 = time.time()
                lazyObj, gcsCuts = mipLazy(testData, i, timeLimit, CWD)
                t1 = time.time()
                row.append(round(t1-t0, 4))
                row.append(round(lazyObj, 2))
                row.append(gcsCuts)
                
                # Benders
                t, bendersObj, gcsCuts, feasCut, optiCut = \
                    BendersV2(testData, i, timeLimit, CWD)
                for n in t.values():
                    row.append(round(n, 4))
                row.append(round(sum(t.values()), 4))
                row.append(round(bendersObj, 2))
                row.append(gcsCuts)
                row.append(feasCut)
                row.append(optiCut)
                
                # write into csv
                # The reason I didnt use with open() is to allow the 
                # current prograss to be saved when the program
                # is been interrupted
                detailCSV = open(detailCSVName, 'a', newline='')
                writer = csv.writer(detailCSV)
                writer.writerow(row)
                detailCSV.close()
        print('--Completed!!!!!!!!!!!!!!!!!!!!!!!!')



def main():
    # Pick one or more ['P', 'Solomon', 'A', 'B']###############
    testNames = ['P']
    ############################################################
    
    timeLimit = 3600
    runTest(testNames, timeLimit)
    
    
if __name__ == '__main__':
    main()