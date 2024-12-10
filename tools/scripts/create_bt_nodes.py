from pathlib import Path
from xml.dom import minidom
from dataclasses import dataclass
from typing import List
import re

@dataclass
class PortMetaData:
    name: str
    type: str

@dataclass
class NodeMetadata:
    name: str
    node_type: str
    ports: List[PortMetaData]

hpp_fp = Path("src/planning/race_decision_engine/src/bt_nodes")
save_path_file = "src/planning/race_decision_engine/resources/race_decision_engine_bt_nodes.xml"
bt_node_desc_pattern = r"BT::[\w]+Port[\w\<\>:\, ]+\([\w\s\"\:\<\>]+"
bt_node_type_pattern = r"BT::\w+Node"
node_type_xml_map = {
    "ConditionNode" : "Condition",
    "SyncActionNode" : "Action"
}
port_hpp_xml_map = {
    "BidirectionalPort" : "inout_port",
    "InputPort" : "input_port",
    "OutputPort" : "output_port"
}

bt_nodes = []
for fp in list(hpp_fp.glob("*/*.cpp")):
    file_name = fp.stem
    node_name = file_name.replace("_", " ").title().replace(" ", "")
    with open(fp) as node_file:
        node_file_text = node_file.read()
        node_type = node_type_xml_map[re.findall(bt_node_type_pattern, node_file_text)[0].strip().replace("BT::", "")]
        node_descriptions = re.findall(bt_node_desc_pattern, node_file_text)
        port_metadatas = []
        for node_description in node_descriptions:
            type_index = node_description.index("<")
            parameter_index = node_description.index("(")
            port_type = port_hpp_xml_map[node_description[:type_index].replace("BT::","")]
            port_name = node_description[parameter_index+1:].strip().replace('"',"")
            port_metadata = PortMetaData(port_name, port_type)
            port_metadatas.append(port_metadata)
    bt_node = NodeMetadata(node_name, node_type, port_metadatas)
    bt_nodes.append(bt_node)

for bt_node in bt_nodes:
    print(bt_node)

root = minidom.Document()
xml = root.createElement("root")
root.appendChild(xml)

tree_nodes_model = root.createElement("TreeNodesModel")
xml.appendChild(tree_nodes_model)

for bt_node in bt_nodes:
    tree_node_bt = root.createElement(bt_node.node_type)
    tree_node_bt.setAttribute('ID', bt_node.name)
    for port_metadata in bt_node.ports:
        port_bt = root.createElement(port_metadata.type)
        port_bt.setAttribute('name', port_metadata.name)
        port_bt.setAttribute('default', "{" + port_metadata.name + "}")
        tree_node_bt.appendChild(port_bt)
    tree_nodes_model.appendChild(tree_node_bt)

xml_str = root.toprettyxml(indent ="\t")
print(xml_str)

if save_path_file:
    with open(save_path_file, "w") as f:
        f.write(xml_str)