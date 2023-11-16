from PyQt5 import QtWidgets
import sys
import json

class JsonEditor(QtWidgets.QWidget):
    def __init__(self, json_data):
        super().__init__()
        self.json_data = json_data
        self.initUI()

    def initUI(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.edits = {}
        self.create_editors(self.json_data, [])
        self.save_button = QtWidgets.QPushButton('Save JSON', self)
        self.save_button.clicked.connect(self.save_json)
        self.layout.addWidget(self.save_button)
        self.setLayout(self.layout)

    def create_editors(self, element, path):
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
            self.layout.addWidget(label)
            self.layout.addWidget(edit)
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

    def save_json(self):
        with open('output.json', 'w') as file:
            json.dump(self.json_data, file, indent=4)
        QtWidgets.QMessageBox.information(self, 'Success', 'JSON has been saved successfully!')

def main():
    app = QtWidgets.QApplication(sys.argv)
    json_data = {
    "SimMode": "",
    "ClockType": "",
    "ClockSpeed": 1,
    "LocalHostIp": "127.0.0.1",
    "ApiServerPort": 41451,
    "RecordUIVisible": True,
    "LogMessagesVisible": True,
    "ShowLosDebugLines": False,
    "ViewMode": "",
    "RpcEnabled": True,
    "EngineSound": True,
    "PhysicsEngineName": "",
    "SpeedUnitFactor": 1.0,
    "SpeedUnitLabel": "m/s",
    # "Wind": {"X": 0, "Y": 0, "Z": 0},
    # "CameraDirector": {
    #     "FollowDistance": -3,
    #     "X": float('nan'),  # NaN in JSON, float('nan') in Python
    #     "Y": float('nan'),
    #     "Z": float('nan'),
    #     "Pitch": float('nan'),
    #     "Roll": float('nan'),
    #     "Yaw": float('nan')
    # },
    # "Recording": {
    #     "RecordOnMove": False,
    #     "RecordInterval": 0.05,
    #     "Folder": "",
    #     "Enabled": False,
    #     "Cameras": [
    #         {"CameraName": "0", "ImageType": 0, "PixelsAsFloat": False, "VehicleName": "", "Compress": True}
    #     ]
    # },
    # "CameraDefaults": {
    #     "CaptureSettings": [
    #         {
    #             "ImageType": 0,
    #             "Width": 256,
    #             "Height": 144,
    #             "FOV_Degrees": 90,
    #             "AutoExposureSpeed": 100,
    #             "AutoExposureBias": 0,
    #             "AutoExposureMaxBrightness": 0.64,
    #             "AutoExposureMinBrightness": 0.03,
    #             "MotionBlurAmount": 0,
    #             "TargetGamma": 1.0,
    #             "ProjectionMode": "",
    #             "OrthoWidth": 5.12
    #         }
    #     ],
    #     "NoiseSettings": [
    #         {
    #             "Enabled": False,
    #             "ImageType": 0,
    #             "RandContrib": 0.2,
    #             "RandSpeed": 100000.0,
    #             "RandSize": 500.0,
    #             "RandDensity": 2,
    #             "HorzWaveContrib": 0.03,
    #             "HorzWaveStrength": 0.08,
    #             "HorzWaveVertSize": 1.0,
    #             "HorzWaveScreenSize": 1.0,
    #             "HorzNoiseLinesContrib": 1.0,
    #             "HorzNoiseLinesDensityY": 0.01,
    #             "HorzNoiseLinesDensityXY": 0.5,
    #             "HorzDistortionContrib": 1.0,
    #             "HorzDistortionStrength": 0.002
    #         }
    #     ],
    #     "Gimbal": {
    #         "Stabilization": 0,
    #         "Pitch": float('nan'),
    #         "Roll": float('nan'),
    #         "Yaw": float('nan')
    #     },
    #     "X": float('nan'),
    #     "Y": float('nan'),
    #     "Z": float('nan'),
    #     "Pitch": float('nan'),
    #     "Roll": float('nan'),
    #     "Yaw": float('nan'),
    #     "UnrealEngine": {
    #         "PixelFormatOverride": [
    #             {
    #                 "ImageType": 0,
    #                 "PixelFormat": 0
    #             }
    #         ]
    #     }
    # },
    # "OriginGeopoint": {
    #     "Latitude": 47.641468,
    #     "Longitude": -122.140165,
    #     "Altitude": 122
    # },
    # "TimeOfDay": {
    #     "Enabled": False,
    #     "StartDateTime": "",
    #     "CelestialClockSpeed": 1,
    #     "StartDateTimeDst": False,
    #     "UpdateIntervalSecs": 60
    # },
    # "SubWindows": [
    #     {"WindowID": 0, "CameraName": "0", "ImageType": 3, "VehicleName": "", "Visible": False, "External": False},
    #     {"WindowID": 1, "CameraName": "0", "ImageType": 5, "VehicleName": "", "Visible": False, "External": False},
    #     {"WindowID": 2, "CameraName": "0", "ImageType": 0, "VehicleName": "", "Visible": False, "External": False}
    # ],
    # "SegmentationSettings": {
    #     "InitMethod": "",
    #     "MeshNamingMethod": "",
    #     "OverrideExisting": True
    # },
    # "PawnPaths": {
    #     "BareboneCar": {"PawnBP": "Class'/AirSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"},
    #     "DefaultCar": {"PawnBP": "Class'/AirSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'"},
    #     "DefaultQuadrotor": {"PawnBP": "Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'"},
    #     "DefaultComputerVision": {"PawnBP": "Class'/AirSim/Blueprints/BP_ComputerVisionPawn.BP_ComputerVisionPawn_C'"}
    # },
    "Vehicles": {
        # "SimpleFlight": {
        #     "VehicleType": "SimpleFlight",
        #     "DefaultVehicleState": "Armed",
        #     "AutoCreate": True,
        #     "PawnPath": "",
        #     "EnableCollisionPassthrogh": False,
        #     "EnableCollisions": True,
        #     "AllowAPIAlways": True,
        #     "EnableTrace": False,
        #     "RC": {
        #         "RemoteControlID": 0,
        #         "AllowAPIWhenDisconnected": False
        #     },
        #     "Cameras": {
        #         # same elements as CameraDefaults above, key as name
        #     },
        #     "X": float('nan'),
        #     "Y": float('nan'),
        #     "Z": float('nan'),
        #     "Pitch": float('nan'),
        #     "Roll": float('nan'),
        #     "Yaw": float('nan')
        # },
        "PhysXCar": {
            "VehicleType": "PhysXCar",
            "DefaultVehicleState": "",
            "AutoCreate": True,
            "PawnPath": "",
            "EnableCollisionPassthrogh": False,
            "EnableCollisions": True,
            "RC": {
                "RemoteControlID": -1
            },
            "Cameras": {
                "MyCamera1": {
                    # same elements as elements inside CameraDefaults above
                },
                "MyCamera2": {
                    # same elements as elements inside CameraDefaults above
                },
            },
            "X": float('nan'),
            "Y": float('nan'),
            "Z": float('nan'),
            "Pitch": float('nan'),
            "Roll": float('nan'),
            "Yaw": float('nan')
        }
    },
    # "ExternalCameras": {
    #     "FixedCamera1": {
    #         # same elements as in CameraDefaults above
    #     },
    #     "FixedCamera2": {
    #         # same elements as in CameraDefaults above
    #     }
    #     }
    }
    editor = JsonEditor(json_data)
    editor.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()