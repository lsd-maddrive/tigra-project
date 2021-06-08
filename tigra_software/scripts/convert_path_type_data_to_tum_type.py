import re

# 16
def convert_to_tum_format(input_array):

    timestamp = re.findall(r'secs: (.+)', input_array[3].rstrip())[0] + '.' + re.findall(r'nsecs: +(\w.+)', input_array[4].rstrip())[0]
  
    x = re.findall(r'x: (.+)', input_array[8].rstrip())[0]
    y = re.findall(r'y: (.+)', input_array[9].rstrip())[0]
    z = re.findall(r'z: (.+)', input_array[10].rstrip())[0]
    q_x = re.findall(r'x: (.+)', input_array[12].rstrip())[0]
    q_y = re.findall(r'y: (.+)', input_array[13].rstrip())[0]
    q_z = re.findall(r'z: (.+)', input_array[14].rstrip())[0]
    q_w = re.findall(r'w: (.+)', input_array[15].rstrip())[0]


    return f'{timestamp} {x} {y} {z} {q_x} {q_y} {q_z} {q_w}'.rstrip() + '\n'

    
my_arr = []

from_file_name = 'travaled_trajectory.txt'
to_file_name = 'tum_traveled_trajectory.txt'

f = open(from_file_name, 'r')
for i in range(9):
    f.readline()
new_f = open(to_file_name, 'w')
count = 0

for i in f:
    if i == '    - \n':
        if my_arr != []:
            result_str = convert_to_tum_format(my_arr)
            new_f.write(result_str)
        my_arr = []
    else:
        my_arr.append(i)


new_f.close()

