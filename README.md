# swisssmartfarming
Repository containing code related to the *Swiss Smart Farming* Project (*SSF*).

## The Project
Within the context of the project *Swiss Smart Farming* various agricultural datasets were collected. For the collection of the datasets the custom sensor-pod shown here was used:

<img src="docs/imgs/sensors-pod.png" alt="Sensors Pod" width="300"/>

The SSF sensor-pod integrates the following sensors and cameras:
* RGB camera:
* VIS camera:
* NIR camera
* thermal camera:
* Lidar:

## Dependencies
Agricultural datasets captured for this project include RGB, hyperspectral, thermal as well as lidar data.
Datasets were stored as [bagfiles](http://wiki.ros.org/Bags), a file format in [ROS](https://www.ros.org/) for storing *ROS message data*.
The use of the package assumes that you have ROS installed on your system. The package was tested (01/10/2021) under *Ubuntu 20.04 LTS* with the *ROS Noetic Ninjemys* distribution.

The package relies on GDAL for operations with geodata, as well as on other libraries. For the installation of the package to be successful, the following libraries have to be installed on the machine, in addition to the already mentioned *ROS Noetic Ninjemys*.

- `build-essential`
- `libboost-python-dev`
- `libexiv2-dev`
- `libgdal-dev`
- `python-all-dev`

To install those dependencies, enter the following in your terminal:
```bash
sudo apt install -y build-essential libboost-python-dev libexiv2-dev libgdal-dev python-all-dev
```

The package was tested with `Python 3.8.10`, but should work under different versions of `python3` as well. For this reason `python3` (recommended `Python 3.8.10`) must be installed in order to proceed.
It is recommended to install all Python dependencies under a virtual environment. The script `setup.bash` creates a virtual environment `venv` under the root folder of the project and installs all the required dependencies from the `requirements.txt` file. Enter the following in your terminal in order to install the dependencies:

```bash
source setup.bash
```

## Usage
There are two main command line tools that can be used in order to process an SSF-dataset. The first one is `preprocess_bag.py` and the second one is `process_pix4d.py`, both are located under the `processing` module.

`preprocess_bag.py` depends on many packages that were installed with `setup.bash`. In order for them to be available the virtual environment has to be activated with `source venv3/bin/activate`. After activating the virtual environment run `processing/preprocess_bag.py -h` in order to get a description on how to use it.

### preprocess_bag.py
`processing/preprocess_bag.py --bagfile /path/to/bagfile.bag --ref_panel {1,2,3}`

Preprocesses an SSF-rosbag dataset.

Given a rosbag file containing a dataset for the Swiss Smart Farming
Project performs the preprocessing steps on the raw images.
The preprocessed images will be stored under the folder 'frames',
located one level higher than the bagfile.

Preprocessing steps include:

1. for RGB images: save them in 'jpg' format and embed the image
metadata (camera name, focal length, GPS location, ...)

2. for hyperspectral images:
    1. reshape the raw image to an image cube according to the specifications from the xml calibration file
    2. apply a 3x3 median filter in order to smoothen the image
    3. converts the images from 8-bit (0-255) to reflectance (0-100%). For this an image of a

All images captured are
stored under a 'topic'. Check 'cfg/cameras' to see all the cameras used
for the project and the corresponding topic.

If a new camera had to be integrated into the system, a new camera
configuration file need to be added, in order to extract and preprocess
the new camera. Under 'cfg/cameras' add a folder named 'camera-name_camera-type' and
under it a text file with the same name and extension '.cfg'. This file
gives some specifications about the camera. Compare (and copy) e.g.
'cfg/cameras/blackfly_rgb/blackfly_rgb.cfg' in order to create a new
camera configuration file. The 'exp_t_topic' field in the file can be
empty, if the exposure time of the camera is not being recorded.

If the value of the field 'type' is 'hyperspectral', then a second text
file (.xml) is expected under the camera folder. The name of the file
does not matter. This file contains information about the hyperspectral
sensor and is unique for every single sensor produced. It has to be
handed in at the purchase of every hyperspectral camera and a copy need
to be stored together with the camera configuration file in order to
make the hyperspectral processing possible.



### process_pix4d.py

## Datasets Structure
All the SSF datasets have the structure shown in the following diagram. The root
folder name is the name of the field. The dataset is split into dates at which
the flights were carried out. These are saved directly under the root folder.
Under every date-folder the following can be found:
* ``bagfile.bag``: symbolic link to the dataset source
[bagfile](http://wiki.ros.org/Bags "ROS - Bags")
* ``bagfile.info``: result of the command ``rosbag info <bagfile.bag>`` saved
to a text-file
* ``rtk_data.csv``: file containing the recorded RTK-GPS positions / altitudes
* camera-folders: can be ``nir``, ``rgb``, ``thermal`` or ``vis``. ``thermal``
is shown in brackets since not all flights were performed with a thermal camera.
All camera-folders have the same substructure:
    - ``frames``: folder containing the camera frames extracted from the
    bagfile. The file ``img_tstamps.csv`` also stored here contains the
    timestamp of every frame
    - ``field-name_date-1_nir``: folder with the standard
    [Pix4D folder structure](https://support.pix4d.com/hc/en-us/articles/202558649-Project-Folder-Structure "Pix4D - Project Folder Structure").
    All Pix4D outputs (mosaics, point clouds, DSM, ...) are saved under this
    directory
    - ``field-name_thermal.p4d``: standard Pix4D project file that can be
    imported into Pix4D in order to regenerate / modify some outputs

```
field-name
├── date-1
│   ├── bagfile.bag
│   ├── bagfile.info
│   ├── rtk_data.csv
│   ├── nir
│   │   ├── frames
│   │   │   ├── frame-1.tif
│   │   │   ├── frame-n.tif
│   │   │   └── img_tstamps.csv
│   │   ├── field-name_date-1_nir
│   │   └── field-name_date-1_nir.p4d
│   ├── rgb
│   │   ├── frames
│   │   │   ├── frame-1.jpg
│   │   │   ├── frame-n.jpg
│   │   │   └── img_tstamps.csv
│   │   ├── field-name_date-1_rgb
│   │   └── field-name_date-1_rgb.p4d
│   ├── (thermal)
│   │   ├── frames
│   │   │   ├── frame-1.tif
│   │   │   ├── frame-n.tif
│   │   │   └── img_tstamps.csv
│   │   ├── field-name_date-1_vis
│   │   └── field-name_date-1_vis.p4d
│   └── vis
│       ├── frames
│       │   ├── frame-1.tif
│       │   ├── frame-n.tif
│       │   └── img_tstamps.csv
│       ├── field-name_date-1_vis
│       └── field-name_date-1_vis.p4d
└── date-n
    ├── bagfile.bag
    ├── bagfile.info
    ├── rtk_data.csv
    ├── nir
    │   ├── frames
    │   │   ├── frame-1.tif
    │   │   ├── frame-n.tif
    │   │   └── img_tstamps.csv
    │   ├── field-name_date-n_nir
    │   └── field-name_date-n_nir.p4d
    ├── rgb
    │   ├── frames
    │   │   ├── frame-1.jpg
    │   │   ├── frame-n.jpg
    │   │   └── img_tstamps.csv
    │   ├── field-name_date-n_rgb
    │   └── field-name_date-n_rgb.p4d
    ├── (thermal)
    │   ├── frames
    │   │   ├── frame-1.tif
    │   │   ├── frame-n.tif
    │   │   └── img_tstamps.csv
    │   ├── field-name_date-n_thermal
    │   └── field-name_date-n_thermal.p4d
    └── vis
        ├── frames
        │   ├── frame-1.tif
        │   ├── frame-n.tif
        │   └── img_tstamps.csv
        ├── field-name_date-n_vis
        └── field-name_date-n_vis.p4d
```

## Downloads
* [Presentation: Microsoft Smart Farming Workshop](doc/presentations/SSF_Microsoft.pdf)

## See also
* [Hemp-Segmentation](https://github.com/dschori/Hemp-Segmentation)
