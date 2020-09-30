from keras_segmentation.models.unet import vgg_unet
import os

model = vgg_unet(n_classes=3, input_height=512, input_width=512)

imgs_path = "/media/seba/Samsung_2TB/forest-project/qgis/gubler/train/images"
msks_path = "/media/seba/Samsung_2TB/forest-project/qgis/gubler/train/masks/"
ckpts_path = "/media/seba/Samsung_2TB/forest-project/qgis/gubler/train/vgg_unet"

model.train(
    train_images=imgs_path,
    train_annotations=msks_path,
    checkpoints_path=ckpts_path,
    epochs=5,
)

out = model.predict_segmentation(
    inp=os.path.join(imgs_path, "x1021_y1014.png"),
    out_fname=os.path.join(msks_path, "x1021_y1014_predict.png"),
)