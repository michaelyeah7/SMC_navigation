agents_file = open("agents.txt","a")

#number of same scenario
groups_num = 4
column_num = 2
row_num = 2
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
    + str(y_step * row_index + robot_y_init) +' 0.00 0.00])\n'
    human1 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init +1) +' 0.00 180.00])\n'
    human2 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -1) +' 0.00 180.00])\n'
    human3 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init +2) +' 0.00 180.00])\n'
    human4 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -2) +' 0.00 180.00])\n'
    human5 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init +3) +' 0.00 180.00])\n'
    human6 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -3) +' 0.00 180.00])\n'
    human7 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init +4) +' 0.00 180.00])\n'
    human8 = 'agent( pose [' + str(x_step * column_index + human_x_init) + ' ' \
    + str(y_step * row_index + human_y_init -4) +' 0.00 180.00])\n'


    #write into file   
    agents_file.write(robot)
    agents_file.write(human1)
    agents_file.write(human2)
    agents_file.write(human3)
    agents_file.write(human4)
    agents_file.write(human5)
    agents_file.write(human6)
    agents_file.write(human7)
    agents_file.write(human8)