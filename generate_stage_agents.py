agents_file = open("agents.txt","a")

#number of same scenario
groups_num = 9
column_num = 3
row_num = 3
#Needed for group location calculations
group_index = 0 
x_step = 40
y_step = 40

#robot initial positions
robot_x_init = -20
robot_y_init = -20
#human initial positions
human_x_init = robot_x_init + 8
human_y_init = robot_y_init 



for i in range(groups_num):
    group_index = i
    column_index = group_index % column_num
    row_index = group_index / column_num

    #generate strings
    robot = 'agent( pose [' + str(x_step * column_index + robot_x_init) + ' ' \
    + str(y_step * row_index + robot_y_init) +' 0.00 0.00 0.00])\n'
    human1 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init +1) +' 0.00 0.00 180.00])\n'
    human2 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -1) +' 0.00 0.00 180.00])\n'

    #write into file   
    agents_file.write(robot)
    agents_file.write(human1)
    agents_file.write(human2)