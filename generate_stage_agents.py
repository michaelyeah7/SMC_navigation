agents_file = open("agents.txt","a")

#number of same scenario
groups_num = 80
column_num = 20
row_num = 4
#Needed for group location calculations
group_index = 0 
x_step = 10
y_step = 10

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