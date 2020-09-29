# MIT License
#
# Copyright (c) 2019 zhixuhao
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from keras.preprocessing.image import ImageDataGenerator


def adjust_data(img, mask, flag_multi_class, num_classes):
    


def train_generator(batch_size, train_path, image_folder, mask_folder, aug_dict,
    image_color_mode="rgb", mask_color_mode="rgb", 
    image_save_prefix= "image", mask_save_prefix="mask", flag_multi_class=False,
    num_class=2, save_to_dir=None, target_size, seed=1):
    """
    """

    image_datagen = ImageDataGenerator(**aug_dict)
    mask_datagen = ImageDataGenerator(**aug_dict)
    image_generator = image_datagen.flow_from_directory(
        train_path,
        target_size=target_size,
        color_mode=image_color_mode,
        classes=[image_folder],
        class_mode=None,
        batch_size=batch_size,
        seed=seed,
        save_to_dir=save_to_dir,
        save_prefix=image_save_prefix,
    )

    mask_generator = mask_datagen.flow_from_directory(
        train_path,
        target_size=target_size,
        color_mode=mask_color_mode,
        classes=[mask_folder],
        class_mode=None,
        batch_size=batch_size,
        seed=seed,
        save_to_dir=save_to_dir,
        save_prefix=mask_save_prefix,
    )

    train_gen = zip(image_generator, mask_generator)
    for (img, mask) in train_gen:
        img, mask = 

