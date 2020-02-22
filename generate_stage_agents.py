agents_file = open("agents.txt","a")

#number of same scenario
groups_num = 2
column_num = 2
row_num = 1
#Needed for group location calculations
group_index = 0 
x_step = 20
y_step = 20

#robot initial positions
robot_x_init = -100
robot_y_init = 80
#human initial positions
human_x_init = -92
human_y_init = 81



for i in range(groups_num):
    group_index = i
    column_index = group_index % column_num
    row_index = group_index / column_num

    #generate strings
    robot = 'agent( pose [' + str(x_step * column_index + robot_x_init) + ' ' \
    + str(y_step * row_index + robot_y_init) +' 0.00 0.00 0.00])\n'
    human1 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init) +' 0.00 0.00 180.00])\n'
    human2 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -2) +' 0.00 0.00 180.00])\n'

    #write into file   
    agents_file.write(robot)
    agents_file.write(human1)
    agents_file.write(human2)