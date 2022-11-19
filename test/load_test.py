import os

CWD = "YOUR/WORKING/DIR"
Augurat = ['A', 'B', 'P']

def load_test(testSet, test_dir):
    # Returns a dictionary of dictionarys of dictionaries
    allTests = {}
    if testSet in Augurat:
        # Augurat test sets
        test_dir = test_dir + '/' + testSet
        os.chdir(test_dir)
        print('Data set loaction: ', os.getcwd())
        for fileName in os.listdir():
            if fileName[-3:] == 'vrp':
                allTests[fileName[:-4]] = {}
                N = {}
                # print(fileName)
                status = 'pass'
                with open(os.path.join(test_dir, fileName), 'r') as f:
                    file = f.readlines()
                    for line in file:
                        line = line.strip()
                        # Check for coordinate and depot section
                        if line == 'NODE_COORD_SECTION':
                            status = 'coord'
                            # print(status)
                            continue
                        elif line == 'DEMAND_SECTION':
                            status = 'pass'
                        elif line == 'DEPOT_SECTION':
                            status = 'deput'
                            # print(status)
                            continue
                        elif line == '-1':
                            status = 'end'
                            # print(status)
                        
                        if status == 'coord':
                            coord = line.split()
                            # print(coord)
                            name = 'c' + coord[0]
                            N[name] = [int(coord[1]), int(coord[2])]
                            # print(line)
                        elif status == 'deput':
                            deput = 'c' + line
                            # print(N)
                        elif status == 'end':
                            d = N.pop(deput)
                            allTests[fileName[:-4]]['N'] = N
                            Ns = N.copy()
                            Nt = N.copy()
                            Nst = N.copy()
                            Nt['t'] = d
                            Ns['s'] = d
                            Nst['s'] = d
                            Nst['t'] = d
                            allTests[fileName[:-4]]['Ns'] = Ns
                            allTests[fileName[:-4]]['Nt'] = Nt
                            allTests[fileName[:-4]]['Nst'] = Nst
                            break
                   
    elif testSet == 'Solomon':
        test_dir = test_dir + '/' + testSet
        os.chdir(test_dir)
        print('Data set loaction: ', os.getcwd())
        # print(os.listdir())
        for fileName in os.listdir():
            if fileName[-3:] == 'txt':
                allTests[fileName[:-4]] = {}
                N = {}
                # print(fileName)
                status = 'pass'
                with open(os.path.join(test_dir, fileName), 'r') as f:
                    file = f.readlines()
                    for line in file:
                        line = line.strip()
                        # Check for coordinate and depot section
                        if line[:8] == 'CUST NO.':
                            status = 'read'
                            continue
                        
                        if status == 'read' and line != '':
                            coord = line.split()
                            # print(coord)
                            if coord[3] == '0':
                                d = [int(coord[1]), int(coord[2])]
                                continue
                            name = 'c' + coord[0]
                            N[name] = [int(coord[1]), int(coord[2])]
                            # print(line)
                    allTests[fileName[:-4]]['N'] = N
                    Ns = N.copy()
                    Nt = N.copy()
                    Nst = N.copy()
                    Nt['t'] = d
                    Ns['s'] = d
                    Nst['s'] = d
                    Nst['t'] = d
                    allTests[fileName[:-4]]['Ns'] = Ns
                    allTests[fileName[:-4]]['Nt'] = Nt
                    allTests[fileName[:-4]]['Nst'] = Nst
                    
    elif testSet == 'UPS':
        # Not yet implimented
        pass
    else:
        # Should not happen unless the user made a mistake
        return None
    return allTests

# a = load_test('Solomon')