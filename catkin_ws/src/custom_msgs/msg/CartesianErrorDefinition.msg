string id
string type     # pose / joint
uint8 component_num     # if pose, is xyzrpy (0 - 5). if joint, is joint number (0 - 7 for kinova)
float32 desired_value   
float32 task_scale
