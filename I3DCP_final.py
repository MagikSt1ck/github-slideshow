import random
import copy
import numpy as np

p_Data=[[-406,23,277],
        [-403,108,256],
        [-392,66,236],
        [-494,18,204],
        [-467,103,204],
        [-447,62,190],
        [-486,144,190],
        [-490,193,190],
        [-715,-208,147],
        [-689,-170,147],
        [-635,0,146],
        [-648,182,146],
        [-447,62,102],
        [-486,144,102],
        [-490,193,102],
        [-494,18,89],
        [-467,103,88],
        [-392,66,56],
        [-403,108,36],
        [-406,23,16]]

a_Matrix= [[0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [1,0,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
           [0,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
           [0,0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0],
           [0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0],
           [0,0,0,0,0,0,1,0,0,1,0,1,0,1,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0],
           [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0],
           [0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0],
           [0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,1,0,0,1],
           [0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,0,1,0],
           [0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1],
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,0]]                     #input point set and adjacency matrix


#get Model Lowest Point
def get_start_point(input_list):             
    z_Value = []
    for i in range(len(input_list)):
        z_Value.append(input_list[i][2])
    pos = z_Value.index(min(z_Value))
    return input_list[pos]


def cal_angle_of_vector(v1, v2=[0,0,1]):
    '''
    print path and vertical vector angle range within 60-90
    '''
    v1_1 = []
    v1_1.append(v1[0])
    v1_1.append(v1[1]) 
    v1_1.append(v1[2])           
    dot_product = np.dot(v1_1, v2)
    length_a = np.linalg.norm(v1_1)
    length_b = np.linalg.norm(v2)
    cos_θ = dot_product / (length_a * length_b)
    if cos_θ >= -0.5:
        return 1
    else:
        return 0


def data_preprocess():
    '''
    If a point has a vertical upward path and is also connected to other points, 
    you need to print the other paths first and the vertical upward paths last
    This function is used to find the point pairs in the structure for this special case
    '''
    data_pair = []
    special_point = []
    for i in range(len(p_Data)):
        for j in range(len(p_Data)):
            if (
                p_Data[i][0] == p_Data[j][0] and 
                p_Data[i][1] == p_Data[j][1] and 
                p_Data[i][2] != p_Data[j][2] and 
                a_Matrix[i][j]
                ):
                element = []
                element.append(p_Data[i])
                element.append(p_Data[j])
                a = sorted(element,key=lambda x:x[2])
                if a not in data_pair:
                    data_pair.append(a)
                else:
                    continue
    for point in data_pair:
        special_point.append(point[0])
    return special_point,data_pair


def path_acquisition():
    '''
    Extracts all print paths in the structure(connected pairs of points)
    The output list contains all the print paths in the structure
    '''
    global path_list
    copy_matrix_1 = copy.deepcopy(a_Matrix)
    path_list = []
    for i in range(len(p_Data)):
        for j in range(len(p_Data)):
            if copy_matrix_1[i][j]:
                point_pair = []
                point_pair.append(p_Data[i])
                point_pair.append(p_Data[j])
                path_list.append(point_pair)
                copy_matrix_1[i][j] = 0
                copy_matrix_1[j][i] = 0
    return path_list    


def path_collision_detection(path1, path2):
    """
    Determine whether the projections of two line segments in space in the xy plane have intersection points            
    return: If there is an intersection, return the coordinates of the intersection (x, y), otherwise return None
    The intersection point of the projection must be in the middle of the line segment, not at any end point
    :param l1: line 1, a list of two points, e.g. [(x1, y1), (x2, y2)]
    :param l2: segment 2, a list of two points, e.g. [(x3, y3), (x4, y4)] 
    """
    l1,l2 = [],[]
    list1,list2,list3,list4 = [],[],[],[]
    list1.append(path1[0][0])
    list1.append(path1[0][1])
    list2.append(path1[1][0])
    list2.append(path1[1][1])
    l1.append(list1)
    l1.append(list2)
    list3.append(path2[0][0])
    list3.append(path2[0][1])
    list4.append(path2[1][0])
    list4.append(path2[1][1])
    l2.append(list3)
    l2.append(list4) #Path two-dimensionality

    def is_point_on_line(point, line_point1, line_point2):
        """
        :Determines if two line segments in the plane have intersection, True if they have intersection, False if they don't
        :param l1: line 1, a list of two points, e.g. [(x1, y1), (x2, y2)]
        :param l2: segment 2, a list of two points, e.g. [(x3, y3), (x4, y4)]             
        :return: If there is an intersection, return the coordinates of the intersection (x, y), otherwise return None
        """
        # Converting a list to a tuple
        point = tuple(point)
        line_point1 = tuple(line_point1)
        line_point2 = tuple(line_point2)
        # Determine if a point is on a straight line
        if (line_point2[1] - line_point1[1]) * (point[0] - line_point1[0]) == \
        (point[1] - line_point1[1]) * (line_point2[0] - line_point1[0]):
            return True
        else:
            return False

    #Whether paths are co-linear or not
    if l1[0][0]==l1[1][0] and l2[0][0]==l2[1][0] and l1[0][0]==l2[0][0]:            #Vertical and co-linear
        return True
    elif l1[0][0]!=l1[1][0] and l2[0][0]!=l2[1][0]:
        if is_point_on_line(l1[0],l2[0],l2[1]) and is_point_on_line(l1[1],l2[0],l2[1]):     #Not vertical and co-linear
            return True
    if len([k for k in l1 if k in l2])==1:  #Projected line segments have an intersection point that is not an interference case
        return False

    # Calculate the slope and y-intercept of the line segment
    def get_equation(p1, p2):
        if p1[0] == p2[0]:
            return None, p1[0]
        else:
            k = (p2[1] - p1[1]) / (p2[0] - p1[0])
            b = p1[1] - k * p1[0]
            return k, b

    # Calculate the intersection point
    def get_intersection_point(l1, l2):
        k1, b1 = get_equation(l1[0], l1[1])
        k2, b2 = get_equation(l2[0], l2[1])
        if k1 is None and k2 is None:
            if l1[0]==l2[0]:
                return True
            else:             # Both lines are perpendicular to the x-axis and cannot intersect
                return False
        elif k1 is None:
            x = b1
            y = k2 * x + b2
        elif k2 is None:
            x = b2
            y = k1 * x + b1
        elif k1 == k2:
            return False  # Two line segments are parallel and cannot intersect
        else:
            x = (b2 - b1) / (k1 - k2)
            y = k1 * x + b1
        # Determine if the intersection point is on two line segments
        if (x < min(l1[0][0], l1[1][0]) or x > max(l1[0][0], l1[1][0]) or
            x < min(l2[0][0], l2[1][0]) or x > max(l2[0][0], l2[1][0]) or
            y < min(l1[0][1], l1[1][1]) or y > max(l1[0][1], l1[1][1]) or
            y < min(l2[0][1], l2[1][1]) or y > max(l2[0][1], l2[1][1])):
            return False  # The intersection points are not on the two line segments
        else:
            return x, y
        
    # Determine if a line segment has an intersection
    intersection_point = get_intersection_point(l1, l2)
    if intersection_point!=False:
        inter_point=list(intersection_point)
        if (
            inter_point!=l1[0] and inter_point!=l1[1] and
            inter_point!=l2[0] and inter_point!=l2[1]
        ):
            return True
    else:
        return False


def interference_path_extraction():
    '''
    Find all the interference path below a certain path and store it in the list [res], 
    the table header element is the certain path
    '''
    global res
    res = []
    for i in range(len(path_list)):
        col_path = []
        for j in range(i+1,len(path_list)):
            if path_collision_detection(path_list[i],path_list[j]):
                col_path.append(path_list[j])
            else:
                continue
        col_path.insert(0,path_list[i])
        res.append(col_path)
    return res


def path_feasibility(point1,point2):
    '''
    Path feasibility detection, 
    return 0 means there is interference, not printable
    '''
    global res

    #Generate input path
    path = []
    pos1 = p_Data.index(point1)
    pos2 = p_Data.index(point2)
    if pos1>pos2:
        path.append(point2)
        path.append(point1)
    else:
        path.append(point1)
        path.append(point2)
    
    for item in res:    #Extracting the print path
        if item[0]==path:
            col_pair = item
            break

    if len(col_pair)==1:    #No interference
        return 1
    else:
        for col_path in col_pair:
            if col_path==path:
                continue
            else:
                p1 = p_Data.index(col_path[0])  #Determine if the interference path of the path has been printed
                p2 = p_Data.index(col_path[1])
            if a_Matrix_cp[p1][p2]==0:
                continue
            else:
                return 0   


# get output point
def DFS():
    global a_Matrix_cp
    a_Matrix_cp = copy.deepcopy(a_Matrix)
    p_out = [get_start_point(p_Data)]
    special_point = data_preprocess()[0]
    z_return = p_Data[0][2]+100
    array = a_Matrix_cp
    while np.all(array == 0) == False:
        pointList = []
        tempPoint = p_out[-1]
        pointPos_1 = p_Data.index(tempPoint)
        for i in reversed(range(len(p_Data))):
            if (
                a_Matrix_cp[pointPos_1][i] and 
                cal_angle_of_vector(np.array(p_Data[i])-np.array(tempPoint)) and  #Print feasibility judgment
                path_feasibility(tempPoint,p_Data[i]) != 0
                ):                                              
                pointList.append(p_Data[i])
        if tempPoint in special_point and len(pointList) != 1:  #Determine if the current point is a special point
            for item in data_preprocess()[1]:
                if item[0] == tempPoint:
                    pointList.remove(item[1])
                    break
        if pointList:
            nextPoint = random.choice(pointList)
            p_out.append(nextPoint)
            pointPos_2 = p_Data.index(nextPoint)
            a_Matrix_cp[pointPos_1][pointPos_2] = 0
            a_Matrix_cp[pointPos_2][pointPos_1] = 0                           
        else:
            array_1 = np.array(a_Matrix_cp)
            if np.all(array_1 == 0):
                return_point = []
                return_point.append(tempPoint[0])
                return_point.append(tempPoint[1])
                return_point.append(z_return)
                p_out.append(return_point)
                return p_out
            else:
                returnPoint_1,returnPoint_2 = [],[]
                returnPoint_1.append(tempPoint[0])
                returnPoint_1.append(tempPoint[1])
                returnPoint_1.append(z_return)
                p_out.append(returnPoint_1)   
                for i in reversed(range(len(p_Data))):
                    array_2 = np.array(a_Matrix_cp[i])
                    if np.all(array_2 == 0) == False:                                     
                        returnPoint_2.append(p_Data[i][0])
                        returnPoint_2.append(p_Data[i][1])
                        returnPoint_2.append(z_return)
                        p_out.append(returnPoint_2)
                        p_out.append(p_Data[i])
                        break
    return p_out                

#----------------------------------Optimization of the objective function

def sum_layer(input_seq):
    """
    Number of sequence stratification
    """
    layer_num = 0  
    for i in range(len(input_seq)):
        if i+1 == len(input_seq):
            break
        else:
            node = input_seq[i] 
            next_node = input_seq[i+1]
            if node in p_Data and next_node not in p_Data:
                layer_num += 1
    return layer_num

def avg_node_repetition(input_seq):
    """
    Average number of node repetitions
    """
    repeat_num = 0
    for i in range(len(input_seq)):
        listA = input_seq[:i-1]
        if input_seq[i] in p_Data and input_seq[i] in listA:
            repeat_num += 1
        else:
            continue
    avg_num = repeat_num / len(p_Data)
    return avg_num

#--------------------------------Constraints on optimization

def max_node_repetition(input_seq):
    """
    Maximum number of node repetitions
    """
    count_times = []
    listA = list(input_seq)
    for i in listA :
        if i in p_Data:
            count_times.append(listA.count(i))
        else:
            continue
    max_time = max(count_times)
    if max_time<5:
        return 1
    else:
        return 0


def DFS_recursion():
    global sum_seq
    sum_seq = []
    num = 1
    while num <= 5:
        a = DFS()
        sum_seq.append(a)
        num+=1
    return sum_seq

def optimize_output(input_seq):
    global new_list
    seqList = copy.deepcopy(input_seq)
    new_list = []
    for i in range(len(seqList)):
        list1 = []
        opt_value = 5*sum_layer(seqList[i])+3*avg_node_repetition(seqList[i])
        list1.append(i)
        list1.append(opt_value)
        new_list.append(list1)
    new_list.sort(key=lambda x:x[-1])
    for item in new_list:
        if max_node_repetition(seqList[item[0]]):
            return seqList[i]
        else:
            continue    
    return new_list

def converter(p_out):
    res = ''
    for point in p_out:
        for coordinate in point:
            res += str(coordinate)+', '
        res = res[:-2]
        res += ' \n'
    return res

def run():
    path_acquisition()
    interference_path_extraction()
    data_preprocess()
    print(converter((optimize_output(DFS_recursion()))))


if __name__=='__main__':
    run()
    print('OK')
