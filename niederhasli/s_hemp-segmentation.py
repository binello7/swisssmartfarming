#!../venv/bin/python2

from config import Config
import os
import keras_segmentation.models.unet

model = keras_segmentation.models.unet.vgg_unet(n_classes=2, input_height=256,
    input_width=256)
config = Config()

if not os.path.isdir(config.checkpoints_path):
    os.mkdir(config.checkpoints_path)

model.train(
    train_images = config.train_images_path,
    train_annotations = config.train_annotations_path,
    checkpoints_path = config.checkpoints_path,
    epochs = 10
)
