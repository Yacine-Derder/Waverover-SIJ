import xml.etree.ElementTree as ElementTree
from pathlib import Path


def dependencies(package_name):
    package_xml = Path(__file__).parents[2] / package_name / 'package.xml'
    root = ElementTree.parse(package_xml).getroot()
    return {
        element.text
        for element in root
        if element.tag in ('depend', 'exec_depend')
    }


def test_end_trial_message_dependency_is_declared_by_both_packages():
    assert 'std_msgs' in dependencies('waverover_waypoint_ui')
    assert 'std_msgs' in dependencies('waverover_controller')
