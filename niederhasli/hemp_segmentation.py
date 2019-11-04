import matplotlib.pyplot as plt
import keras_segmentation as k_seg
import cv2
import numpy as np

mask_path = "/home/seba/MATLAB/niederhasli/PixelLabelData/Label_1.png"
img_path = "/media/seba/Samsung_2TB/Analysis/Image.segmentation/images/image.png"

mask = cv2.imread(mask_path)
mask = mask[:,:,2]
mask = mask - 1
mask[mask==255] = 0
print(np.max(mask))


cv2.imwrite("/media/seba/Samsung_2TB/Analysis/Image.segmentation/annotations_train/image.png", mask)

model = k_seg.models.unet.vgg_unet(n_classes=3, input_height=416, input_width=608)

model.train(
    train_images = "/media/seba/Samsung_2TB/Analysis/Image.segmentation/images_train",
    train_annotations = "/media/seba/Samsung_2TB/Analysis/Image.segmentation/annotations_train",
    checkpoints_path = "/media/seba/Samsung_2TB/Analysis/Image.segmentation/tmp/vgg_unet_1",
    epochs=5
)

out = model.predict_segmentation(
    inp="/media/seba/Samsung_2TB/Analysis/Image.segmentation/images_test/image.png",
    out_fname="/media/seba/Samsung_2TB/Analysis/Image.segmentation/tmp/out.png"
)

plt.imshow(out)
#
#

# img = cv2.imread(img_path)
# print(img.shape)
#
# img_resized = cv2.resize(img, (200, 200))
#
# print(img_resized.shape)
#
# plt.imshow(img)
# plt.imshow(mask, alpha=0.5)
# plt.show()
