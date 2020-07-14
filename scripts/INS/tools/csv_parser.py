import numpy as np
import csv
import sys

"""
    CSV Library to read comma seperated lists from generic CSV files and ROS Bag CSV exports
"""

def readCSV(fileName, parameters = -1):
    """Reads a list of comma seperated data (numbers) from a given text/csv file

    :param fileName: Name of the file to be read
    :param parameters: Number of values per one row (Optional parameter)
                       If not provided, will read all the values in a row, by default.
    :return [status, datalist, length]
            Status -  Boolean value Success / Fail status of reading the file (True = Success)
            dataList - List of values. 
                        Each item in the list will be, a list of values in one row in the input file
            length - Number of items in the dataList

    Example Input:
        # This is an example input file
        1,2,3
        4,5,6
        
    Output:
        [True,[[1,2,3],[4,5,6]], 2]
    """


    status = False
    datalist = []

    try:
        with open(fileName) as csvFile:
            for line in csvFile:
                # Ignore commented lines
                line = line.partition('#')[0]
                line = line.rstrip()
                # Ignore whitespaces and split by comma delimeters
                lineItems = line.replace(' ','').split(',')
                # Ignore empty lines
                if(len(lineItems) == 1 and lineItems[0] == ''): continue
                # Check number of parameters per line, and append to output list
                if(parameters == -1 or parameters == len(lineItems)):
                    try:
                        lineValues = [i for i in lineItems] 
                        datalist.append(lineValues)
                    except:
                        print ("Error in line : ", line)
            # Successfully read the File
            status = True
        
    except:
        print ("File reading error in ", fileName, " : ", sys.exc_info()[0])
        if(parameters != -1):
            print ("Required number of parameters: ", parameters)

    return status, datalist, len(datalist)    

def readROSBagCSV (fileName, fields = None, dtype=None):
    """Reads a list of fields from a ROS Bag CSV file
       The ROS Bag CSV file must contain a field definition line, starting with '%' symbol.

    :param fileName: Name of the file to be read
    :param fields: List of values to read from the file. If not provided, will read all fields
    :param dtype: Numpy dtype of each data column, for type conversion. If not provided, all data  columns 
                    will be returned as S20 (20 Character String Fields)
    :return [status, data]
            Status -  Boolean value Success / Fail status of reading the file (True = Success)
            data - Numpy structured array of requested data
    :note1 Use rostopic echo -b "${filevar}.bag" -p $topicvar > "${filevar}.csv" to generate the CSV file

    :note2 Use x.view((float, len(x.dtype.names))) to type convert output to a regular numpy array if suitable

    Example Input:
        % time, x, y
        1,2,3
        4,5,6

        readROSBagCSV (fileName, fields=["x","y"])
        
    Output: (Numpy Structured Array)
        [[2,3],
         [5,6]]
    """
    status, dataList, dataLines = readCSV(fileName)

    if (status != True):
        print ("Error reading file ", fileName)
        return status, None

    #dataList dimensions : lineCount x lineWordCount
    if(dataList[0][0][0] != '%'):
        print ("Invalid ROS Bag CSV file. Field definition line not found. Use rostopic echo -b '${filevar}.bag' -p $topicvar > '${filevar}.csv' to generate the CSV file")
        return False, None

    # Accounting for 1st line (Field Definition)
    dataLines = dataLines - 1
    fieldList = dataList.pop(0)
    # Removing '%' symbol from first element
    fieldList[0] = fieldList[0][1:]

    # if fields list is not provided, return all fields
    if (fields == None):
        fields = fieldList

    # Check validity of field count and given data
    fieldCount = len(fields)
    if (fieldCount == 0):
        print ("Invalid fields. Number of fields requested should be minimum 1")
        return False, None

    # Read required data field indexes
    fieldCount = len(fields)
    indexes = []
    for f in fields:
        try:
            indexes.append(fieldList.index(f))
        except ValueError:
            print ("ROS Bag CSV Reading Error: Field '", f, "' not found in ", fileName, " file")
            return False, None

    # Read data and append to output matrix
    data = np.zeros(dataLines, dtype={'names':tuple(fields), 'formats':tuple(['S20' for i in (fields)])})
    for id in range(dataLines):
        data[id] = tuple([dataList[id][i] for i in indexes])

    # Cast types if dtype is provided
    if (dtype != None):
        dataMat = np.zeros(dataLines, dtype={'names':tuple(fields), 'formats':tuple(dtype)})
        for fieldID in range(fieldCount):
            fieldName = fields[fieldID]
            dataType = dtype[fieldID]
            #dataMat[fieldName] = np.asarray(data[fieldName], dataType)   
            dataMat[fieldName] = data[fieldName].astype(dataType)
        data = dataMat

    return True, data

def writeCSV(data, fileName, fields=None):
    """Writes a 2D array to a CSV file as a list of comma seperated data (numbers)

    :param fileName: Name of the file to be written
    :param fields: List of field names (column descriptions).
                    These will be written as a comma seperated comment line at the begining of the file, starting with '%'
    :param data: 2D Numpy array with shape lineCount x lineDataCount

    Example Input:
        data = [[1,2,3],[4,5,6]]
        fields = ['x', 'y', 'z']
        fileName = output.csv
        
    output.csv:
        %x,y,z
        1,2,3
        4,5,6
    """
    try:
        with open(fileName, mode='w') as output:
            if fields != None :
                output.write("%" + ','.join(str(field) for field in fields) + '\n')
            for line in range(data.shape[0]):
                output.write(','.join(str(value) for value in data[line,:]) + '\n')
            print ("Data written to " + fileName)
    except:
            print ("Output file writing error: ", sys.exc_info()[0])
