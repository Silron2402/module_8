import os
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import matplotlib.pyplot as plt 
    

class WorldsGenerator:
    def __init__(self, xml_file: str) -> None:
        self.xml_file = xml_file
        self.tree = ET.parse(xml_file)
        self.root = self.tree.getroot()
        with open(xml_file, 'r') as file:
            self.original_content = file.read()
        self.new_models = []

    def add_gazebo_model(self, name: str, uri: str,
                     position: list = [0, 0, 0, 0, 0, 0],
                     velocity: list = [0, 0, 0, 0, 0, 0],
                     acceleration: list = [0, 0, 0, 0, 0, 0],
                     scale: list = [1, 1, 1]):
        # Create the include element
        include_element = ET.Element('include')
        
        # Add the include's sub-elements
        name_element = ET.SubElement(include_element, 'name')
        name_element.text = name
        
        uri_element = ET.SubElement(include_element, 'uri')
        uri_element.text = uri
        
        include_pose_element = ET.SubElement(include_element, 'pose')
        include_pose_element.text = ' '.join(map(str, position))
        
        scale_element = ET.SubElement(include_element, 'scale')
        scale_element.text = ' '.join(map(str, scale))
        
        velocity_element = ET.SubElement(include_element, 'velocity')
        velocity_element.text = ' '.join(map(str, velocity))
        
        acceleration_element = ET.SubElement(include_element, 'acceleration')
        acceleration_element.text = ' '.join(map(str, acceleration))
        
        # Find the world element and append the new include element
        world_element = self.root.find(".//world[@name='default']")
        world_element.append(include_element)
        self.new_models.append(include_element)
        

    def format_model_element(self, model_element):
        formatted_element = '<include>\n'
        for child in model_element:
            formatted_element += '    <{}>{}</{}>\n'.format(child.tag, child.text, child.tag)
        formatted_element += '</include>'
        return formatted_element

    
    def save(self, output_file: str):
        # Convert new models to string with proper indentation
        new_models_str = '\n'.join([self.format_model_element(model) for model in self.new_models])

        # Find the place to insert new models
        insertion_idx = self.original_content.rfind("</world>")
        content = self.original_content[:insertion_idx] + new_models_str + self.original_content[insertion_idx:]

        # Save to the output file
        with open(output_file, "w") as f:
            f.write(content)


    def add_flat_plane():
        pass
    

if __name__ == "__main__":
    script_directory = os.path.dirname(os.path.abspath(__file__))
    generator = WorldsGenerator(script_directory+"/../sim/worlds/empty_base.world")
    
    model_name = 'new_model'
    model_position = [1.0, 2.0, 3.0, 0, 0, 0]
    model_velocity = [0, 0, 0, 0, 0, 0]
    model_acceleration = [0, 0, 0, 0, 0, 0]
    
    generator.add_gazebo_model("my_model", "Pine Tree")
    generator.add_gazebo_model("my_model1", "Pine Tree")
    generator.add_gazebo_model("my_model2", "Pine Tree")
    generator.save(script_directory+"/../sim/worlds/test_gen.world")
