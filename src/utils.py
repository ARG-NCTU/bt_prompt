import os
import re


class TreeNode:
    def __init__(self, name):
        self.name = name
        self.node_type = "unknown"
        self.children = []
        self.parent = "unknown"

    def set_type(self, node_type):
        self.node_type = node_type

    def set_parent(self, parent):
        self.parent = parent

    def get_information(self):
        return self.name, self.parent, self.node_type

    def get_all_nodes(self, node, children_list=None):
        if children_list is None:
            children_list = []
        if node:
            for child in node.children:
                children_list.append(child.name)
                self.get_all_nodes(child, children_list)
        return children_list
    
    def get_childs(self, node):
        children_list = []
        if node:
            for child in node.children:
                children_list.append(child.name)
        return children_list

    def insert_node_only(self, child):
        new_node = TreeNode(child)
        self.children.append(new_node)
        return new_node  

    def insert(self, child, node_type):
        new_node = TreeNode(child)
        new_node.set_type(node_type)
        self.children.append(new_node)
        return new_node   


def process_find_subtrees(text):
    # Base Case: If the text contains only commands, process and return them
    commands = re.findall(r'\"[^\"]+\"', text)
    return commands

def extract_procedure(text):
    # Extract content between "Procedure:{...}"
    procedure_match = re.search(r'Procedure:\{(.*?)\}$', text, re.DOTALL)
    if not procedure_match:
        return None
    return procedure_match.group(1).strip()

def extract_phase(layer, text):
    pattern = re.compile(r'Do phase (' + r'\d+-'*(layer-1) + r'\d+)' # captures index according to the specified layer depth
                         r'(?: if (' + r'\d+-'*(layer-1) + r'\d+)? (success|failed))?'  # captures conditionals like "if 1 success" or "if 1-1 failed"
                         r'(?: at the same time)?\s*:\{(.*?)\}', re.DOTALL)  # captures content inside {}
    results = []

    for match in pattern.finditer(text):
        phase_info = {
            'index': match.group(1),
            'conditional_index': match.group(2) if match.group(2) else None,
            'conditional_status': match.group(3) if match.group(3) else None,
            'content': match.group(4).strip()
        }

        # Check for "at the same time" condition
        if "at the same time" in match.group(0):
            phase_info['at_same_time'] = True
        else:
            phase_info['at_same_time'] = False

        phase_info['minimized_subtree'] = False

        results.append(phase_info)
    
    return len(results), results

def classify_group(input_struct):
    # Group dictionaries based on the first parts of their indices
    groups = {}
    for item in input_struct:
        key_parts = item['index'].split('-')
        key = '-'.join(key_parts[:-1])  # Use all parts except the last one as the key
        if key not in groups:
            groups[key] = []
        groups[key].append(item)

    output = list(groups.values())

    return output

def update_content(subtree_in_same_root, command_list):
    for subtree in subtree_in_same_root:
        for potential_node in subtree:
            if detect_string('Do', potential_node['content']) or detect_string('phase', potential_node['content'])  or detect_string('Procedure', potential_node['content']):
                potential_node['minimized_subtree'] = False
            else:
                potential_node['minimized_subtree'] = True
                potential_node['content'] = check_and_cut(potential_node['content'], command_list)
    return subtree_in_same_root

def detect_dash(s):
    return '-' in s

def detect_string(substring, main_string):
    return substring in main_string

def check_and_cut(input_string, string_list):
    substrings_found = []

    # Repeat until the input string doesn't contain any string in the list
    while any(sub in input_string for sub in string_list):
        for sub in string_list:
            while sub in input_string:
                input_string = input_string.replace(sub, '', 1)
                substrings_found.append(sub)
    
    return substrings_found

def string_to_variable(input_string):
    """Create a variable from a string."""
    # Generate a consistent "random" hash for the input string
    hashed_string = hashlib.sha256(input_string.encode()).hexdigest()
    # Use only first 8 characters to make the variable name shorter
    variable_name = "var_" + hashed_string[:8]
    # Set the variable to some value (e.g., the original string) globally
    globals()[variable_name] = input_string
    return variable_name


def create_order(input_struct):
    # Group nodes based on the first part of the index

    groups = {}
    for node in input_struct:
        # key = node['index'].split('-')[0]
        if detect_dash(node['index']):
            key = node['index'][:len(node['index'])-2]
        else:
            key = '0'
        if key not in groups:
            groups[key] = []
        groups[key].append(node)
    
    # Sort the nodes in each group based on the index
    for key in groups:
        groups[key].sort(key=lambda x: [int(i) for i in x['index'].split('-')])
    
    # Create the root for each group and append to the output
    output = []
    # relation = []
    for key, group_nodes in groups.items():
        root_index = key
        order = []
        for node in group_nodes:
            order.append((key, node['index']))
        output.append({'root_index': root_index, 'order': order})

    return output

def append_subtree(tree_nodes, id, existing_string, tab_size):
    local_string = ""

    for i in range(tab_size):
        local_string += "\t"

    node_name, node_parent, node_type = tree_nodes[id].get_information()
    
    if node_type == '<sequence>':   
        local_string += "->\n"

    elif node_type == '<fallback>':   
        local_string += "?\n"

    elif node_type == '<parallel>':  
        local_string += "||\n"

    else:
        local_string += node_type[0]
        local_string += "\n"


    # print("origin string: ", local_string)
    # print("local string: ", existing_string)

    existing_string += local_string
    # print("final string: ", existing_string)
    # subtree_string = local_string + existing_string

    if len(tree_nodes[id].get_childs(tree_nodes[id])) == 0:
        return existing_string
    else:
        for tree in tree_nodes[id].get_childs(tree_nodes[id]):
            existing_string = append_subtree(tree_nodes, tree, existing_string, tab_size + 1)
        return existing_string

    

def output_tree_config(tree_nodes, tree_list):
    # for node in tree_list:
    #     node_name, node_parent, node_type = tree_nodes[node].get_information()
    #     print("------get_information_reply------")
    #     print("ID: ", node_name)
    #     print("Parent: ", node_parent)
    #     print("Type: ", node_type)
        
    #     print(tree_nodes[node].get_childs(tree_nodes[node]))
    #     print("---------------------------------")

    text = ""

    node_name, node_parent, node_type = tree_nodes['0'].get_information()
    if node_parent == 'unknown' and node_name == '0':
        if node_type == '<sequence>':   
            text += "->\n"
        elif node_type == '<fallback>':   
            text += "?\n"
        elif node_type == '<parallel>':  
            text += "||\n"    
    

    for node in tree_nodes['0'].get_childs(tree_nodes['0']):
        node_name, node_parent, node_type = tree_nodes[node].get_information()
        
        text += "\t"

        if node_type == '<sequence>':   
            text += "->\n"
        elif node_type == '<fallback>':   
            text += "?\n"
        elif node_type == '<parallel>':  
            text += "||\n"
        
        for tree in tree_nodes[node].get_childs(tree_nodes[node]):
            text = append_subtree(tree_nodes, tree, text, 2)
        
    print(text)
    return text

    

def merge_without_duplicated(list_1, list_2):
    result = list_1.copy()
    for item in list_2:
        if item not in result:
            result.append(item)
    return result

def find_dic_by_value(dict_list, target_key, target_value):
    for index, dictionary in enumerate(dict_list):
        if dictionary.get(target_key) == target_value:
            return index
    return None

def find_list_index(input_list, target_key):
    result = []
    start_idx = None

    for i, value in enumerate(input_list):
        if value == target_key:
            if start_idx is None:
                start_idx = i
        else:
            if start_idx is not None:
                result.append((start_idx, i - 1))  
                start_idx = None

    if start_idx is not None:
        result.append((start_idx, len(input_list) - 1))
    
    if len(result) == 0:
        result = None
    return result

def parsing(input_text):
    nesting_layer_num = 1
    root_list = []
    tree_list = []

    procedure_text = extract_procedure(input_text)

    command_list = process_find_subtrees(procedure_text)

    while True:
        _, subtree_in_same_layer = extract_phase(nesting_layer_num, procedure_text)
        subtree_in_same_root = classify_group(subtree_in_same_layer)
        subtree_in_same_root = update_content(subtree_in_same_root, command_list)
        subtree_num = len(subtree_in_same_root)

        # print("-----current layer-------------")
        # print(nesting_layer_num)

        if subtree_num == 0:
            break
        else:
            nesting_layer_num += 1
        
        # print("-----current tree number-------")
        # print(subtree_num)

        # print("-----tree order----------------")
        # print("---") 
        for subtree in subtree_in_same_root:
            same_root_trees = create_order(subtree)
            for tree in same_root_trees:
                root_list.append(tree)
                # print(tree)

        # print("---")    
        for subtree in subtree_in_same_root:
            for tree in subtree:
                tree_list.append(tree)
                # print(tree) 

    return root_list, tree_list
        
def create_tree(root_list, tree_list):
    dynamic_index = -1
    index_list = []
    nodes = {}
    parent_register = None
    for root in root_list:
        relations = root['order']
        logic_list = []
        for parent, child in relations:
            parent_register = parent
            if parent not in nodes:
                nodes[parent] = TreeNode(parent)
                index_list.append(parent)
            if child not in nodes:
                nodes[child] = TreeNode(child)
                index_list.append(child)
                index = find_dic_by_value(tree_list, 'index', child)
                
                # A parallel node
                if tree_list[index]['at_same_time'] == True and tree_list[index]['minimized_subtree'] == True:
                    for parallel_nodes in  tree_list[index]['content']:
                        nodes[str(dynamic_index)] = TreeNode(str(dynamic_index))
                        nodes[str(dynamic_index)].set_type([parallel_nodes])
                        nodes[str(dynamic_index)].set_parent(child)
                        nodes[child].children.append(nodes[str(dynamic_index)])
                        nodes[child].set_type('<parallel>')
                        index_list.append(str(dynamic_index))
                        dynamic_index -= 1
                # A subtree node
                elif tree_list[index]['minimized_subtree'] == True:
                    nodes[child].set_type(tree_list[index]['content'])
                    nodes[child].set_parent(parent)
                # else(only set the root)
                else:
                    nodes[child].set_parent(parent)

                logic_list.append((tree_list[index]['conditional_status']))

                
            nodes[parent].children.append(nodes[child])

            
        
        # decide the logic flow
        fallback_list = find_list_index(logic_list, 'failed')
        sequence_list = find_list_index(logic_list, 'success')

        print("current root:", parent_register)
        print("current logic", logic_list)
        print("fallback: ", fallback_list)
        print("sequence: ", sequence_list)
        print("Judge: ")

        if len(logic_list) == 1 and fallback_list == None and sequence_list == None:
            nodes[parent_register].set_type('<sequence>')
            # print('single as sequence')

        elif fallback_list == None:
            nodes[parent_register].set_type('<sequence>')
            # print('single sequence')

        elif sequence_list == None:
            nodes[parent_register].set_type('<fallback>')
            # print('single fallback')

        elif fallback_list != None and sequence_list != None and len(fallback_list) == 1 and len(sequence_list) == 1:
            nodes[parent_register].set_type('<sequence>')

            children_list = nodes[parent_register].get_childs(nodes[parent_register])

            start_bit = fallback_list[0][0] - 1
            end_bit = fallback_list[0][1]
            # print("list: ", children_list)
            # print(type(children_list))

            # print(children_list[start_bit])
            # print(children_list[end_bit])

            # new node
            nodes[str(dynamic_index)] = TreeNode(str(dynamic_index))
            nodes[str(dynamic_index)].set_type('<fallback>')
            nodes[str(dynamic_index)].set_parent(parent_register)
            nodes[str(dynamic_index)].children.append(nodes[children_list[start_bit]])
            nodes[str(dynamic_index)].children.append(nodes[children_list[end_bit]])

            # add parent
            nodes[children_list[start_bit]].set_parent(str(dynamic_index))
            nodes[children_list[end_bit]].set_parent(str(dynamic_index))

            # remove old child from root
            nodes[parent_register].children.remove(nodes[children_list[start_bit]])
            nodes[parent_register].children.remove(nodes[children_list[end_bit]])

            # add new node to old root
            nodes[parent_register].children.append(nodes[str(dynamic_index)])

            index_list.append(str(dynamic_index))
            
            dynamic_index -= 1


        elif len(fallback_list) > 1 or len(sequence_list) > 1:
            print('the most challanging...')
        
        else:
            print("Something goes wrong...")
               


        # print("-----\n")

    return nodes, index_list

def remove_front_until(text, specified_text):
    index = text.find(specified_text)
    if index != -1:
        return text[index:]
    else:
        return text


def find_file(file_name, directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if os.path.splitext(file)[0] == file_name:
                return os.path.join(root, file)
    return None

def append_tab(input_string, num_tabs):
    new_strings = ""
    strings = input_string.split("\n") 
    for string in strings:
        if string == "->" or string == "?":
            prefix = ""
            for i in range(num_tabs):
                prefix += '\t'
            new_strings = new_strings + string
        else:
            prefix = "\n"
            for i in range(num_tabs):
                prefix += '\t'
            new_strings = new_strings + prefix + string
    
    return new_strings

def subtree_assembly(raw, path):
    lines = raw.split("\n")  # Split the input string by newlines
    result = []

    for line in lines:
        if line.startswith("\t"):
            num_tabs = line.count("\t")  # Count the number of tabs in the line
            stripped_line = line.strip()  # Remove leading and trailing whitespaces
            if stripped_line.startswith("\"") and stripped_line.endswith("\""):
                extracted_string = stripped_line[1:-1]  # Extract the string between quotation marks
                result.append((extracted_string, num_tabs))
            else:
                extracted_string = stripped_line[0:2]
                result.append((extracted_string, num_tabs))

    # print(result)

    final = "->"
    # final = ""
    for string, num_tabs in result:
        final = final + "\n"
        if string != "->" and string != "||" and string != "?":
            directory_path = path + string + ".tree"

            with open(directory_path, 'r') as file:
                subtree = file.read().rstrip()
            new_subtree = append_tab(subtree, num_tabs)

            for i in range(num_tabs):
                final = final + "\t"
            final = final + new_subtree
        else:
            for i in range(num_tabs):
                final = final + "\t"
            final = final + string
    
    return final



def post_processing(raw_string):
    root_list, tree_list = parsing(raw_string)
    nodes, index_list = create_tree(root_list, tree_list)
    root = nodes['0']
    return_list = root.get_all_nodes(nodes['0'])
    final_list = merge_without_duplicated(index_list, return_list)
    output_text = output_tree_config(nodes, final_list)
    return output_text