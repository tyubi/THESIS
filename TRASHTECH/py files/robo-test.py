# from roboflow import Roboflow


# # !pip install roboflow

# # from roboflow import Roboflow
# # rf = Roboflow(api_key="PTpUisRvjOjvKvqQ0PUq")
# # project = rf.workspace("ecompdetection").project("electronics-component-detection")
# # version = project.version(2)
# # dataset = version.download("yolov8")
                

# rf = Roboflow(api_key="PTpUisRvjOjvKvqQ0PUq")
# version = rf.workspace("ecompdetection").project("electronics-component-detection").version(2, local="http://localhost:9001/")

# prediction = version.model.predict("Electronics Component Detection.v2i.yolov8/test/images/IMG_0344_mov-0012_jpg.rf.608614f1e75e3035993e340a944c8364.jpg")
# print(prediction.json())


from roboflow import Roboflow

rf = Roboflow(api_key="PTpUisRvjOjvKvqQ0PUq")

project = rf.workspace("ecompdetection").project("electronics-component-detection")

model = project.version(2, local="http://localhost:9001").model

prediction = model.predict("Electronics Component Detection.v2i.yolov8/test/images/IMG_0344_mov-0012_jpg.rf.608614f1e75e3035993e340a944c8364.jpg")

print(prediction.json())