from hmi.routeEdit.gui import RouteEditWindow
from PySide6 import QtWidgets
import logger
import sys

log = logger.setup_app_level_logger(file_name="app_debug.log")


file_paths = {
    "corridor": (
        "networkFiles/corridor/corridor.net.xml",
        "networkFiles/corridor/corridor.rou.xml",
    ),
    "CarlaTown05": (
        "networkFiles/CarlaTown05/Town05.net.xml",
        "networkFiles/CarlaTown05/carlavtypes.rou.xml,networkFiles/CarlaTown05/Town05.rou.xml",
    ),
    "CarlaTown10": (
        "networkFiles/CarlaTown10/Town10.net.xml",
        "networkFiles/CarlaTown10/carlavtypes.rou.xml,networkFiles/CarlaTown10/Town10.rou.xml",
    ),
    "bigInter": (
        "networkFiles/bigInter/bigInter.net.xml",
        "networkFiles/bigInter/bigInter.rou.xml",
    ),
    "roundabout": (
        "networkFiles/roundabout/roundabout.net.xml",
        "networkFiles/roundabout/roundabout.rou.xml",
    ),
    "bilbao":   (
        "networkFiles/bilbao/osm.net.xml",
        "networkFiles/bilbao/osm.rou.xml",
    ),
    #######
    # Please make sure you have request the access from https://github.com/ozheng1993/UCF-SST-CitySim-Dataset and put the road network files (.net.xml) in the relevent networkFiles/CitySim folder
    "freewayB": (
        "networkFiles/CitySim/freewayB/freewayB.net.xml",
        "networkFiles/CitySim/freewayB/freewayB.rou.xml",
    ),
    "Expressway_A": (
        "networkFiles/CitySim/Expressway_A/Expressway_A.net.xml",
        "networkFiles/CitySim/Expressway_A/Expressway_A.rou.xml",
    ),
    ########
}

if __name__ == "__main__":
    net_file, rou_file = file_paths['CarlaTown10']

    app = QtWidgets.QApplication([])
    gui = RouteEditWindow(net_file, rou_file, "0")
    gui.show()

    sys.exit(app.exec())
