# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arean.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!
import rospkg
import pathlib
import os
from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox


def get_ros_package_path(package_name: str) -> str:
    try:
        import rospkg

        rospack = rospkg.RosPack()
        return rospack.get_path(package_name)
    except:
        return ""


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setFixedSize(590, 549)
        MainWindow.move(100, 100)
        MainWindow.setWindowTitle("arena-rosnav-3D")
        font = QtGui.QFont()
        font.setPointSize(12)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(30, 20, 501, 451))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pedsimSceneLabel = QtWidgets.QLabel(self.layoutWidget)
        self.pedsimSceneLabel.setObjectName("pedsimSceneLabel")
        self.pedsimSceneLabel.setFont(font)
        self.gridLayout.addWidget(self.pedsimSceneLabel, 11, 0, 1, 1)
        self.actorHeightLabel = QtWidgets.QLabel(self.layoutWidget)
        self.actorHeightLabel.setEnabled(True)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.actorHeightLabel.setFont(font)
        self.actorHeightLabel.setObjectName("actorHeightLabel")
        self.gridLayout.addWidget(self.actorHeightLabel, 9, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.pedvisCheckBox = QtWidgets.QCheckBox(self.layoutWidget)
        self.pedvisCheckBox.setText("")
        self.pedvisCheckBox.setChecked(True)
        self.pedvisCheckBox.setObjectName("pedvisCheckBox")
        self.gridLayout.addWidget(self.pedvisCheckBox, 12, 2, 1, 1)
        self.plannerLabel = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.plannerLabel.setFont(font)
        self.plannerLabel.setObjectName("plannerLabel")
        self.gridLayout.addWidget(self.plannerLabel, 3, 0, 1, 1)
        self.mapLabel = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.mapLabel.setFont(font)
        self.mapLabel.setObjectName("mapLabel")
        self.gridLayout.addWidget(self.mapLabel, 7, 0, 1, 1)
        self.actorsBox = QtWidgets.QSpinBox(self.layoutWidget)
        self.actorsBox.setProperty("value", 3)
        self.actorsBox.setObjectName("actorsBox")
        self.gridLayout.addWidget(self.actorsBox, 6, 2, 1, 1)
        self.WorldLabel = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.WorldLabel.setFont(font)
        self.WorldLabel.setObjectName("WorldLabel")
        self.gridLayout.addWidget(self.WorldLabel, 0, 0, 1, 1)
        self.TaskBox = QtWidgets.QComboBox(self.layoutWidget)
        self.TaskBox.setEnabled(True)
        self.TaskBox.setStatusTip("")
        self.TaskBox.setFrame(True)
        self.TaskBox.setObjectName("TaskBox")
        self.TaskBox.addItem("")
        self.TaskBox.addItem("")
        self.TaskBox.addItem("")
        self.TaskBox.currentIndexChanged.connect(self.taskChanged)
        self.gridLayout.addWidget(self.TaskBox, 1, 2, 1, 1)
        self.actorHeightBox = QtWidgets.QDoubleSpinBox(self.layoutWidget)
        self.actorHeightBox.setMinimum(1.1)
        self.actorHeightBox.setSingleStep(0.1)
        self.actorHeightBox.setProperty("value", 1.1)
        self.actorHeightBox.setObjectName("actorHeightBox")
        self.gridLayout.addWidget(self.actorHeightBox, 9, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 5, 0, 1, 1)
        self.actorsLabel = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.actorsLabel.setFont(font)
        self.actorsLabel.setObjectName("actorsLabel")
        self.gridLayout.addWidget(self.actorsLabel, 6, 0, 1, 1)
        self.pedsimLabel = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.pedsimLabel.setFont(font)
        self.pedsimLabel.setObjectName("pedsimLabel")
        self.gridLayout.addWidget(self.pedsimLabel, 10, 0, 1, 1)
        self.plannerBox = QtWidgets.QComboBox(self.layoutWidget)
        self.plannerBox.setObjectName("plannerBox")
        self.plannerBox.addItem("")
        self.plannerBox.addItem("")
        self.plannerBox.addItem("")
        self.plannerBox.addItem("")
        self.plannerBox.addItem("")
        self.gridLayout.addWidget(self.plannerBox, 3, 2, 1, 1)
        self.mapLine = QtWidgets.QLineEdit(self.layoutWidget)
        self.mapLine.setToolTipDuration(1)
        self.mapLine.setAutoFillBackground(False)
        self.mapLine.setObjectName("mapLine")
        self.gridLayout.addWidget(self.mapLine, 7, 2, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 12, 0, 1, 1)
        self.worldBox = QtWidgets.QComboBox(self.layoutWidget)
        self.worldBox.setObjectName("worldBox")
        self.worldBox.currentTextChanged.connect(self.worldChanged)
        self.gridLayout.addWidget(self.worldBox, 0, 2, 1, 1)
        self.pedsimCheckBox = QtWidgets.QCheckBox(self.layoutWidget)
        self.pedsimCheckBox.setText("")
        self.pedsimCheckBox.setChecked(True)
        self.pedsimCheckBox.setObjectName("pedsimCheckBox")
        self.gridLayout.addWidget(self.pedsimCheckBox, 10, 2, 1, 1)
        self.comboBox = QtWidgets.QComboBox(self.layoutWidget)
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.gridLayout.addWidget(self.comboBox, 5, 2, 1, 1)
        self.mapBrowseButton = QtWidgets.QPushButton(self.layoutWidget)
        self.mapBrowseButton.setEnabled(True)
        self.mapBrowseButton.setObjectName("mapBrowseButton")
        self.mapBrowseButton.clicked.connect(self.onMapBrowseClicked)
        self.gridLayout.addWidget(self.mapBrowseButton, 7, 3, 1, 1)
        self.pedsimSceneLine = QtWidgets.QLineEdit(self.layoutWidget)
        self.pedsimSceneLine.setObjectName("pedsimSceneLine")
        self.gridLayout.addWidget(self.pedsimSceneLine, 11, 2, 1, 1)
        self.pedsimSceneButton = QtWidgets.QPushButton(self.layoutWidget)
        self.pedsimSceneButton.setObjectName("pedsimSceneButton")
        self.pedsimSceneButton.clicked.connect(self.onSceneBrowseClicked)
        self.gridLayout.addWidget(self.pedsimSceneButton, 11, 3, 1, 1)
        self.scenarioLabel = QtWidgets.QLabel(self.layoutWidget)
        self.scenarioLabel.setEnabled(True)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.scenarioLabel.setFont(font)
        self.scenarioLabel.setObjectName("scenarioLabel")
        self.gridLayout.addWidget(self.scenarioLabel, 2, 0, 1, 1)
        self.scenarioLine = QtWidgets.QLineEdit(self.layoutWidget)
        self.scenarioLine.setEnabled(True)
        self.scenarioLine.setObjectName("scenarioLine")
        self.gridLayout.addWidget(self.scenarioLine, 2, 2, 1, 1)
        self.scenarioButton = QtWidgets.QPushButton(self.layoutWidget)
        self.scenarioButton.setEnabled(True)
        self.scenarioButton.setObjectName("scenarioButton")
        self.scenarioButton.clicked.connect(self.onScenarioBrowseClicked)
        self.gridLayout.addWidget(self.scenarioButton, 2, 3, 1, 1)
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(140, 480, 301, 41))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.startButton = QtWidgets.QPushButton(self.widget)
        self.startButton.setObjectName("startButton")
        self.startButton.clicked.connect(self.onStartClicked)
        self.horizontalLayout.addWidget(self.startButton)
        self.stopButton = QtWidgets.QPushButton(self.widget)
        self.stopButton.setObjectName("stopButton")
        self.stopButton.clicked.connect(self.onStopClicked)
        self.horizontalLayout.addWidget(self.stopButton)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.empty_scene = (
            get_ros_package_path("simulator_setup")
            + "/scenarios/ped_scenarios/empty.xml"
        )
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        self.pedsimSceneLabel.setStatusTip(
            _translate(
                "MainWindow",
                "Specify path to a ped scene file in xml format. Used only for obstacles",
            )
        )
        self.pedsimSceneLabel.setText(_translate("MainWindow", "Pedsim scene"))
        self.actorHeightLabel.setStatusTip(
            _translate("MainWindow", "Specify fixed actor height/Z-position.")
        )
        self.actorHeightLabel.setText(_translate("MainWindow", "Actor height"))
        self.label.setText(_translate("MainWindow", "Task mode"))
        self.plannerLabel.setText(_translate("MainWindow", "Local planner"))
        self.mapLabel.setStatusTip(
            _translate(
                "MainWindow",
                "Specify extra map to use for obstacle management. Leave empty to use robot's map",
            )
        )
        self.mapLabel.setText(_translate("MainWindow", "Additional map"))
        self.WorldLabel.setText(_translate("MainWindow", "World"))
        self.TaskBox.setItemText(1, _translate("MainWindow", "random"))
        self.TaskBox.setItemText(0, _translate("MainWindow", "scenario"))
        self.TaskBox.setItemText(2, _translate("MainWindow", "manual"))
        self.actorHeightBox.setStatusTip(
            _translate("MainWindow", "Specify fixed actor height/Z-position.")
        )
        self.label_2.setText(_translate("MainWindow", "Robot model"))
        self.actorsLabel.setStatusTip(
            _translate("MainWindow", "Set the amount of pedsim actors")
        )
        self.actorsLabel.setText(_translate("MainWindow", "Actors"))
        self.pedsimLabel.setText(_translate("MainWindow", "Enable pedsim"))
        self.plannerBox.setItemText(0, _translate("MainWindow", "teb"))
        self.plannerBox.setItemText(1, _translate("MainWindow", "dwa"))
        self.plannerBox.setItemText(2, _translate("MainWindow", "mpc"))
        self.plannerBox.setItemText(3, _translate("MainWindow", "rlca"))
        self.plannerBox.setItemText(4, _translate("MainWindow", "cadrl"))
        self.mapLine.setToolTip(
            _translate("MainWindow", "Specify extra map to use for obstacle management")
        )
        self.mapLine.setStatusTip(
            _translate(
                "MainWindow",
                "Specify extra map to use for obstacle management. Leave empty to use robot's map",
            )
        )
        self.mapLine.setPlaceholderText(
            _translate("MainWindow", "Path to map.yaml file")
        )
        self.label_3.setText(_translate("MainWindow", "Enable pedvis"))
        self.comboBox.setItemText(0, _translate("MainWindow", "burger"))
        self.comboBox.setItemText(1, _translate("MainWindow", "waffle_pi"))
        self.comboBox.setItemText(2, _translate("MainWindow", "waffle"))
        self.mapBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.pedsimSceneLine.setStatusTip(
            _translate(
                "MainWindow",
                "Specify path to a ped scene file in xml format. Used only for obstacles",
            )
        )
        self.pedsimSceneButton.setText(_translate("MainWindow", "Browse"))
        self.scenarioLabel.setStatusTip(
            _translate("MainWindow", "Path to a scenario in json format")
        )
        self.scenarioLabel.setText(_translate("MainWindow", "Scenario file"))
        self.scenarioLine.setStatusTip(
            _translate("MainWindow", "Path to a scenario in json format")
        )
        self.scenarioButton.setText(_translate("MainWindow", "Browse"))
        self.startButton.setText(_translate("MainWindow", "Start arena"))
        self.stopButton.setText(_translate("MainWindow", "Stop simulation"))
        rospack = rospkg.RosPack()
        folder = pathlib.Path(rospack.get_path("simulator_setup") + "/worlds")
        map_folders = [p for p in folder.iterdir() if p.is_dir()]
        names = [p.parts[-1] for p in map_folders]
        for i, name in enumerate(names):
            self.worldBox.addItem(name)
        self.worldBox.setCurrentText("small_warehouse")

    def onMapBrowseClicked(self):
        f = QtWidgets.QFileDialog.getOpenFileName(
            MainWindow,
            "Select Map yaml",
            get_ros_package_path("simulator_setup") + "/maps",
            filter="yaml(*.yaml)",
        )
        if os.path.isfile(f[0]):
            self.mapLine.setText(f[0])

    def onScenarioBrowseClicked(self):
        f = QtWidgets.QFileDialog.getOpenFileName(
            MainWindow,
            "Select Scenario json",
            get_ros_package_path("simulator_setup") + "/scenarios",
            filter="json(*.json)",
        )
        if os.path.isfile(f[0]):
            self.scenarioLine.setText(f[0])

    def onSceneBrowseClicked(self):
        f = QtWidgets.QFileDialog.getOpenFileName(
            MainWindow,
            "Select Pedsim scene",
            get_ros_package_path("simulator_setup") + "/scenarios/ped_scenarios",
            filter="xml(*.xml)",
        )
        if os.path.isfile(f[0]):
            self.pedsimSceneLine.setText(f[0])

    def onStartClicked(self):
        import subprocess
        import shlex
        import os

        world = f"world:={self.worldBox.currentText()}"
        model = f"model:={self.comboBox.currentText()}"
        planner = f"local_planner:={self.plannerBox.currentText()}"
        task_mode = f"task_mode:={self.TaskBox.currentText()}"
        actor_height = f"actor_height:={str(self.actorHeightBox.value())}"
        actors = f"actors:={self.actorsBox.textFromValue(self.actorsBox.value())}"
        pedsim = f"enable_pedsim:={str(self.pedsimCheckBox.isChecked()).lower()}"
        pedvis = f"enable_pedvis:={str(self.pedvisCheckBox.isChecked()).lower()}"
        if self.pedsimSceneLine.text() != "":
            scene = f"scene_file:={self.pedsimSceneLine.text()}"
        else:
            scene = f"scene_file:={self.empty_scene}"
        scenario_file = self.scenarioLine.text()
        if scenario_file != "":
            scenario_file = (
                f"scenario_file:={os.path.basename(self.scenarioLine.text())}"
            )
        else:
            if self.TaskBox.currentText() == "scenario":  # no scenario file selected
                msg = QMessageBox()
                msg.setWindowTitle("Error")
                msg.setText("No scenario file specified, can not start the simulation.")
                x = msg.exec()
                return
        obs_map = self.mapLine.text()
        if obs_map != "":
            obs_map = f"additional_map:=true add_map_path:={self.mapLine.text()}"
        command = f"interminal --script 'workon rosnav; roslaunch arena_bringup start_arena_gazebo.launch {world} {model} {planner} {task_mode} {actor_height} {actors} {pedsim} {pedvis} {obs_map} {scenario_file} {scene}'"
        self.p = subprocess.Popen(shlex.split(command))

    def onStopClicked(self):
        import subprocess

        if self.p != None:
            subprocess.Popen(
                "killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient",
                shell=True,
            )
            subprocess.Popen("rosnode kill --all", shell=True)
            self.p.kill()
            msg = QMessageBox()
            msg.setWindowTitle("Warning")
            msg.setText(
                "Please wait a bit before starting new simulation, as it may take some time for the sim to gracefully terminate."
            )
            x = msg.exec()

    def taskChanged(self, index):
        if index == 0:
            self.scenarioLabel.setVisible(True)
            self.scenarioLine.setVisible(True)
            self.scenarioButton.setVisible(True)
        else:
            self.scenarioLabel.setVisible(False)
            self.scenarioLine.setVisible(False)
            self.scenarioButton.setVisible(False)

    def worldChanged(self, world):
        import os.path

        # Setting default scenario for this world if it exists
        sim_path = get_ros_package_path("simulator_setup")
        scenario_file = sim_path + f"/scenarios/{world}.json"
        if os.path.isfile(scenario_file):
            self.scenarioLine.setText(sim_path + f"/scenarios/{world}.json")
        else:
            self.scenarioLine.setText("")

        # Setting default scene for this world if it exists
        scene_file = sim_path + f"/scenarios/ped_scenarios/{world}.xml"
        if os.path.isfile(scene_file):
            self.pedsimSceneLine.setText(scene_file)
        else:
            self.pedsimSceneLine.setText("")


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
