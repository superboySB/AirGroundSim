from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import json

class JsonEditor(QtWidgets.QWidget):
    def __init__(self, json_data):
        super().__init__()
        self.json_data = json_data
        self.setWindowTitle("仿真参数配置界面")
        self.resize(1920, 1080)
        self.initUI()

    def initUI(self):
        # Create a scroll area to handle many fields
        scroll = QtWidgets.QScrollArea()
        widget = QtWidgets.QWidget()
        self.layout = QtWidgets.QGridLayout(widget)

        # Set style
        self.setStyleSheet("QWidget { font-size: 14px; } QPushButton { width: 200px; height: 40px; }")

        self.edits = {}
        self.create_editors(self.json_data, [])
        
        self.save_button = QtWidgets.QPushButton('Save JSON', self)
        self.save_button.clicked.connect(self.open_save_dialog)
        self.layout.addWidget(self.save_button, self.layout.rowCount(), 0, 1, 2)
        
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        
        # Main layout
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def create_editors(self, element, path):
        row = self.layout.rowCount()
        if isinstance(element, dict):
            for key, value in element.items():
                self.create_editors(value, path + [key])
        elif isinstance(element, list):
            for i, item in enumerate(element):
                self.create_editors(item, path + [i])
        else:
            label = QtWidgets.QLabel('/'.join(map(str, path)))
            edit = QtWidgets.QLineEdit(str(element))
            edit.textChanged.connect(lambda value, path=path: self.update_json(path, value))
            col = len(path) % 2  # Arrange in two columns
            self.layout.addWidget(label, row + col, 0)
            self.layout.addWidget(edit, row + col, 1)
            self.edits[tuple(path)] = edit

    def update_json(self, path, value):
        element = self.json_data
        for key in path[:-1]:
            element = element[key]
        try:
            # Convert to original data type
            value = type(element[path[-1]])(value)
        except ValueError:
            pass  # Handle the case where value is empty string and cannot be converted to original type
        element[path[-1]] = value

    def open_save_dialog(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,"Save JSON","","JSON Files (*.json)", options=options)
        if fileName:
            self.save_json(fileName)

    def save_json(self, fileName):
        with open(fileName, 'w') as file:
            json.dump(self.json_data, file, indent=4)
        QtWidgets.QMessageBox.information(self, 'Success', f'JSON has been saved successfully to {fileName}!')

def main():
    app = QtWidgets.QApplication(sys.argv)
    json_data = {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
        "SettingsVersion": 1.2,
        "ClockType": "SteppableClock",
        "ClockSpeed": 1,
        "SimMode": "Car",
        "Vehicles": {
            "Vehicle1": {
                "VehicleType": "PhysXCar",
                "AutoCreate": True,
                "Sensors": {
                    "Imu": {
                        "SensorType": 2,
                        "Enabled": True
                    },
                    "LidarSensor1": {
                        "SensorType": 6,
                        "Enabled": True,
                        "NumberOfChannels": 16,
                        "RotationsPerSecond": 10,
                        "PointsPerSecond": 288000,
                        "X": 0,
                        "Y": 0,
                        "Z": -1,
                        "Roll": 0,
                        "Pitch": 0,
                        "Yaw": 0,
                        "Range": 100,
                        "VerticalFOVUpper": 15,
                        "VerticalFOVLower": -15,
                        "HorizontalFOVStart": 0,
                        "HorizontalFOVEnd": 360,
                        "DrawDebugPoints": True,
                        "DataFrame": "SensorLocalFrame",
                        "UpdateFrequency": 10,
                        "NoiseExist": True,
                        "AngleStd": 0.1,
                        "RangeStd": 10,
                        "MotionDistortion": True,
                        "RotationDistortion": True
                    }
                },
                "Cameras": {
                    "front_center_custom": {
                        "CaptureSettings": [
                            {
                                "PublishToRos": 1,
                                "ImageType": 0,
                                "Width": 1080,
                                "Height": 720,
                                "FOV_Degrees": 27,
                                "DepthOfFieldFstop": 2.8,
                                "DepthOfFieldFocalDistance": 200.0,
                                "DepthOfFieldFocalRegion": 200.0,
                                "TargetGamma": 1.5
                            }
                        ],
                        "X": 1.75,
                        "Y": 0,
                        "Z": -1.25,
                        "Pitch": 0,
                        "Roll": 0,
                        "Yaw": 0
                    }
                }
            }
        }
    }
    editor = JsonEditor(json_data)
    editor.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()