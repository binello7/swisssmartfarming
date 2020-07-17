from glob import glob
from rasterio import MemoryFile
from rasterio.warp import Resampling, reproject
from rootpath import detect
from warnings import warn
import geopandas as gpd
import itertools
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
        self.nodata = -10000
        self.dates = []
        self.datasets_name = []
        self.datasets = {}
        self.shapefile = None
#-------------------------------------------------------------------------------

    def add_dataset(self, dataset_path):
        name_splits = dataset_path.split(self._sep)[-1].split('_')

        date = name_splits[0]
        flight = name_splits[1]
        if date not in self.dates:
            self.dates.append(date)

        camera = '_'.join(name_splits[2:4])
        dataset_name = '_'.join((date, flight, camera))
        if dataset_name not in self.datasets_name:
            self.datasets_name.append(dataset_name)
            xml_file = glob(os.path.join(self._cfgpath, camera, '*.xml'))[0]
            self.datasets.update({dataset_name:
                {'dataset': rio.open(dataset_path),
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

    def crop_merge_write_visnir(self, outputs_path):
        try:
            shapes = [feature["geometry"] for _, feature in
                self.shapefile.iterrows()]
        except AttributeError:
            raise AttributeError("No shapefile found. Please add a shapefile.")
        else:
            # check if directory exists. Create it if not
            if not os.path.isdir(outputs_path):
                os.makedirs(outputs_path)

            for date in self.dates:
                same_date = [name for name in self.datasets_name
                    if date in name]
                print(same_date)

                vis = self.datasets[same_date[1]]['dataset']
                print(vis.res)
                nir = self.datasets[same_date[0]]['dataset']
                print(nir.res)
                wl_vis = self.datasets[same_date[1]]['wavelengths']
                wl_nir = self.datasets[same_date[0]]['wavelengths']
                wls = np.concatenate((wl_vis, wl_nir))
                vis_profile = vis.profile
                nir_profile = nir.profile

                nir_data, nir_transform = riom.mask(nir, shapes, crop=True)
                nir_profile = nir_profile.update(transform=nir_transform,
                    height=nir_data.shape[1], width=nir_data.shape[2])

                vis_data, vis_transform = riom.mask(vis, shapes, crop=True)
                vis_profile = vis_profile.update(transform=vis_transform,
                    height=vis_data.shape[1], width=vis_data.shape[2])

                merged_name = date + '_vis-nir.tif'
                dst_path = os.path.join(outputs_path, merged_name)


                kwargs = vis.meta.copy()
                kwargs.update({
                    'width': vis_data.shape[2],
                    'height': vis_data.shape[1],
                    'count': vis_data.shape[0] + nir_data.shape[0],
                    'transform': vis_transform
                    })

                nir_data, nir_transform = reproject(
                    source=nir_data,
                    destination=np.empty((nir_data.shape[0], vis_data.shape[1],
                        vis_data.shape[2])),
                    src_transform=nir_transform,
                    src_crs=nir.crs,
                    src_nodata=self.nodata,
                    dst_transform=vis_transform,
                    dst_crs=vis.crs,
                    resampling=Resampling.bilinear
                )
                assert nir_transform == vis_transform

                visnir = np.concatenate((vis_data, nir_data))
                visnir = visnir.astype(np.dtype('float32'))
                with rio.open(dst_path, 'w', **kwargs) as dst:
                    dst.write(visnir)
#-------------------------------------------------------------------------------

    @staticmethod
    def write_dataset(dir, filename, dataset, wavelengths=None):
        filename = ufunc.add_ext(filename, 'tif')
        filepath = os.path.join(dir, filename)
        profile = dataset.profile
        with rio.open(filepath, 'w', **profile) as dst:
            # Check if wavelengths were given. If not do nothing
            if type(wavelengths) == type(None):
                pass
            else:
                for i, wl in enumerate(wavelengths):
                    dst.update_tags(i+1, WAVELENGTH=str(wl))
            dst.write(dataset.read())
#-------------------------------------------------------------------------------
