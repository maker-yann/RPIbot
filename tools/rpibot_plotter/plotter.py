import matplotlib.pyplot as plt
import csv
import getopt, sys
import time
from datetime import datetime

parameter = []
filename = ""
nbPlots = len(parameter)
x = []
plots = {}
headers = []
hIndexes = {}
rows = 0

def createDummyCSV():
    with open('test.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        writer.writerow(['Timestamp', "x", "y", "z"])
        for i in range(1, 10):
            timestamp = str(datetime.now())
            x = 0.0+i
            y = 0.1
            z = 0.2
            writer.writerow([timestamp, x, y, z])
            time.sleep(0.1)

def headerIndexes(headers):
    i = 0
    for h in headers:
        hIndexes[h] = i
        i += 1
    #print(hIndexes)

if __name__ == "__main__":
    # sshpass -p "raspberry" scp pi@rpibot:/home/pi/rpibot/trace.csv .
    # Run with : "python3 plotter.py -f trace.txt -t timestamp -p pwmL,pwmR"
    
    try:
        opts, args = getopt.getopt(sys.argv[1:], "f:t:p:")
    except getopt.GetoptError as err:
        # print help information and exit:
        print(err) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    for o, a in opts:
        if o == "-f":
            filename = a
        elif o == "-t":
            parameter.append(a)
        elif o in ("-p", "--plot"):
            parameter.extend(a.split(','))
        else:
            assert False, "unhandled option"

    print(parameter)

    with open(filename,'r') as csvfile:
        plotfile = csv.reader(csvfile, delimiter=';')
#        createDummyCSV()
        rowCount = 0
        for p in parameter:
            plots[p] = []
        for row in plotfile:
            #print(row)
            if rowCount > 0:
                for p in parameter:
                    if hIndexes[p] == 0:
                        try:
                            date_time_obj = datetime.strptime(row[hIndexes[p]], '%Y-%m-%d %H:%M:%S.%f')
                            x.append(date_time_obj)
                        except:
                            x.append(float(row[hIndexes[p]]))
                    else:
                        plots[p].append(float(row[hIndexes[p]]))
            else:
                headers = row
                rows = len(row)
                #print("Rows: " + str(rows))
                headerIndexes(headers)
            rowCount += 1
    # plot first graph
    for p in parameter:
        if hIndexes[p] != 0:
            plt.plot(x,plots[p], label=p)
    plt.xlabel('t')
    plt.ylabel('y')
    plt.title('Rpibot')
    plt.legend()
    plt.show()
