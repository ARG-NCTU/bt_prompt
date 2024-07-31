#!/usr/bin/env python3
import os
import rospkg
import rospy
import json

def parse_json2txt(args):
    # get the json file from the path
    for root, dirs, files in os.walk(args.json_dir):
        print(files)
        for cnt, file in enumerate(files):
            try: 
                json_file = args.json_dir + "/" + file
                txt_file = args.txt_dir + "/" + "raw"+ str(cnt) + ".txt"
                print(json_file)
                print(txt_file)

                data = None
                with open(json_file, 'r') as f:
                    if args.model_name == "gpt-4o":
                        lines = f.readlines()
                        try:
                            json_content = "".join(lines[1:-1])
                            data = json.loads(json_content)
                            print(f"successfully loaded JSON file {json_file}")
                        except:
                            print(f"Invalid JSON file {json_file}")
                            pass
                    else:
                        try:
                            data = json.load(f)
                            print(f"successfully loaded JSON file {json_file}")
                        except:
                            print(f"Invalid JSON file {json_file}")
                            pass
                

                initialize = data["initialize"]
                behavior_waypts = data["behavior_waypts"]

                with open(txt_file, 'w') as f:
                    # Write the initialize part
                    f.write("initialize:\n")
                    for init in initialize:
                        f.write("initialize " + init["variable_name"] + " = " + init["variable_value"] + "\n")
                    
                    # Write the behavior_waypts part
                    f.write("\nbehavior_waypts:\n")
                    for behavior in behavior_waypts:
                        f.write("name = " + behavior["name"] + "\n")
                        for condition in behavior["condition"]:
                            f.write("condition = " + condition + "\n")
                        for endflag in behavior["endflag"]:
                            f.write("endflag = " + endflag + "\n")
                        f.write("speed = " + str(behavior["speed"]) + "\n")
                        f.write("repeat = " + str(behavior["repeat"]) + "\n")
                        
                        if (isinstance(behavior["points"], dict)):
                            # format=lawnmower, label=foxtrot, (x,y)=("x of certer point","y of certer point"), height="total height of lawnmower", width="total width of lawnmower", lane_width="lane width", rows="east-west or north-south", (startx, starty)= (x of starting point, y of starting point), degs="Rotation angle"
                            f.write("format= " + str(behavior["points"]["format"]) + "," + "label= " + str(behavior["points"]["label"]) + "," + "(x,y)= " + str(behavior["points"]["(x,y)"]) + "," + "height= " + str(behavior["points"]["height"]) + "," + "width= " + str(behavior["points"]["width"]) + "," + "lane_width= " + str(behavior["points"]["lane_width"]) + "," + "rows= " + str(behavior["points"]["rows"]) + "," + "(startx, starty)= " + str(behavior["points"]["(startx, starty)"]) + "," + "degs= " + str(behavior["points"]["degs"]) + "\n")
                        elif(isinstance(behavior["points"], str)): 
                            f.write("points = " + behavior["points"] + "\n\n")
            except:
                print(f"Invalid JSON file {json_file}")
                continue

def parse_input_file(input_file):
    initialize_lines = []
    behavior_sections = []
    current_section = []
    in_initialize = False
    in_behavior = False

    with open(input_file, 'r') as file:
        for line in file:
            stripped_line = line.strip()
            if stripped_line == 'initialize:':
                in_initialize = True
                in_behavior = False
                continue
            elif stripped_line == 'behavior_waypts:':
                in_initialize = False
                in_behavior = True
                if current_section:
                    behavior_sections.append(current_section)
                    current_section = []
                continue
            
            if in_initialize:
                initialize_lines.append(stripped_line)
            elif in_behavior:
                if stripped_line == '':
                    if current_section:
                        behavior_sections.append(current_section)
                        current_section = []
                else:
                    current_section.append(stripped_line)
                    
        if current_section:
            behavior_sections.append(current_section)
    
    return initialize_lines, behavior_sections

def create_minimal_section(behavior_content):
    minimal_section_template = """Behavior = BHV_Waypoint
{
  %s
  pwt       = 100
  perpetual = true
  speed_alt = 1.2
  use_alt_speed = true
  lead = 8
  lead_damper = 1
  lead_to_start = true
  capture_line = true
  capture_radius = 3.0
  slip_radius = 5.0
  efficiency_measure = all
  order = normal
}\n"""
    return minimal_section_template % ("\n  ".join(behavior_content))

def write_output_file(output_file, initialize_lines, behavior_sections, virtual):
    with open(output_file, 'w') as file:
        for line in initialize_lines:
            file.write(f"{line}\n")
        if virtual is not True:
            v_content = "initialize NAV_X = 0\ninitialize NAV_Y = 0\ninitialize NAV_HEADING = 0\ninitialize NAV_DEPTH = 0\ninitialize NAV_SPEED = 0"
            file.write(v_content)       
        file.write("\n\n")
        
        for behavior_content in behavior_sections:
            minimal_section = create_minimal_section(behavior_content)
            file.write(minimal_section)
            file.write("\n")

def parse_txt2bhv(args):
    # get the txt file from the path
    for root, dirs, files in os.walk(args.txt_dir):
        for cnt, file in enumerate(files):
            bhv_file = args.bhv_dir + "/" + "raw"+ str(cnt) + ".bhv"
            txt_file = args.txt_dir + "/" + file
            initialize_lines, behavior_sections = parse_input_file(txt_file)
            write_output_file(bhv_file, initialize_lines, behavior_sections, args.virtual)

class Args:
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            setattr(self, key, value)

if __name__ == "__main__":
    rospy.init_node("behavior_tree_generation_node")
    rospack = rospkg.RosPack()

    mission_name = rospy.get_param("~mission_name", "test")
    gen_file = rospy.get_param("~gen_file", "json")
    model_name = rospy.get_param("~model_name", "")
    count = rospy.get_param("~count", 30)
    virtual = rospy.get_param("~virtual", True)
    json_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "moos", mission_name, "raw", model_name, gen_file)
    txt_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "moos", mission_name, "raw", model_name, "txt")
    if not os.path.exists(txt_dir):
        os.makedirs(txt_dir)
    bhv_dir = os.path.join(rospack.get_path("behavior_tree_generation"), "config", "moos", mission_name, "raw", model_name, "bhv")
    if not os.path.exists(bhv_dir):
        os.makedirs(bhv_dir)
    

    args = {
        "json_dir": json_dir,
        "gen_file": gen_file,
        "model_name": model_name,
        "txt_dir": txt_dir,
        "bhv_dir": bhv_dir,
        "virtual": virtual
    }
    args = Args(args)

    rospy.loginfo("start to transform json to txt")
    parse_json2txt(args)
    rospy.loginfo("start to transform txt to bhv")
    parse_txt2bhv(args)
    rospy.loginfo("finish the parsing process")