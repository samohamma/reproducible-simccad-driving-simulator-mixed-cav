import re

a_file = open("BmwX5.proto", "r")
list_of_lines = a_file.readlines()
linNum = len(list_of_lines)

for i in range(0,linNum):
    if list_of_lines[i] == "coord Coordinate {" and list_of_lines[i+1] == "point [":
        i=i+2
        if list_of_lines[i].find(","):
            result = re.split(' ', list_of_lines[i])
            for t in range(0,len(result)):
                if t % 3 ==0:
                    result[t]=str(float(result[t])*(-1))
            list_of_lines[i]=" ".join(result)
        else:
            while True:
                result = re.split(' ', list_of_lines[i])
                result[0] = str(float(result[0])*(-1))
                list_of_lines[i]=" ".join(result)
                i +=1
                if list_of_lines[i].find("]"):
                    break

            
a_file = open("BmwX5-left.proto", "w")
a_file.writelines(list_of_lines)
a_file.close()