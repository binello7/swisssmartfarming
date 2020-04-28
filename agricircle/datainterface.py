from glob import glob
from rasterio import MemoryFile
from rasterio.warp import Resampling
from rootpath import detect
from warnings import warn
import geopandas as gpd
import numpy as np
import os
import rasterio as rio
import rasterio.mask as riom
import utils.functions as ufunc

from IPython import embed


class DataInterface:
    def __init__(self):
        self._sep = os.path.sep
        self._rootpath = detect()
        self._cfgpath = os.path.join(self._rootpath, 'cfg/cameras')
        self.location = None
        self.dates = []
        self.datasets_name = []
        self.datasets = {}
        self.shapefile = None
#-------------------------------------------------------------------------------

    def add_dataset(self, dataset_path):
        name_splits = dataset_path.split(self._sep)[-1].split('_')

        location = name_splits[0]
        if self.location == None:
            self.location = location

        elif (self.location != None and self.location != location):
            raise Exception("Location of dataset added is different from "
                "location of the previous datasets. 'DataInterface' is "
                "intended for use with datasets of the same location.")

        date = name_splits[1]
        if date not in self.dates:
            self.dates.append(date)

        camera = '_'.join(name_splits[2:4])
        dataset_name = date + '-' + camera
        if dataset_name not in self.datasets_name:
            self.datasets_name.append(dataset_name)
            xml_file = glob(os.path.join(self._cfgpath, camera, '*.xml'))[0]
            self.datasets.update({dataset_name:
                {'dataset': rio.open(dataset_path),
                'location': location,
                'date': date,
                'camera': camera,
                'wavelengths': ufunc.read_virtualwavelengths(xml_file)
                    .astype(int)
                }
            })

        else:
            warn('Dataset {} already added. Ignoring.'.format(dataset_name))
            return

#-------------------------------------------------------------------------------

    def add_shapefile(self, shapefile_path):
        self.shapefile = gpd.read_file(shapefile_path)
#-------------------------------------------------------------------------------

    def clip_with_shapefile(self):
        """Crops the specified dataset using the previously loaded shapefile
        """

        try:
            shapes = [feature["geometry"] for _, feature in
                self.shapefile.iterrows()]
        except AttributeError:
            raise AttributeError("No shapefile found. Please add a shapefile.")
        else:
            for name, el in self.datasets.items():
                profile = el['dataset'].profile
                data, transform = riom.mask(el['dataset'], shapes, crop=True)
                profile.update(transform=transform, height=data.shape[1],
                    width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[name]['dataset'] = memfile.open()
#-------------------------------------------------------------------------------

    def merge_vis_nir(self, vis, nir):
        height = self.datasets[vis]['dataset'].height
        width = self.datasets[vis]['dataset'].width
        out_shape = (height, width)
        profile = self.datasets[vis]['dataset'].profile

        # ref_array = self.datasets[dataset_name].read(1)
        # self.data_mask = (ref_array!=self.no_data_val) & (ref_array!=0)

        vis_data = self.datasets[vis]['dataset'].read()
        nir_data = self.datasets[nir]['dataset'].read(out_shape=out_shape,
            resampling=Resampling.bilinear)
        data = np.concatenate((vis_data, nir_data), axis=0)
        count = data.shape[0]
        profile.update(count=count)

        return data, profile
#-------------------------------------------------------------------------------
