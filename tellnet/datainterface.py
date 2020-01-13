import warnings
import rasterio as rio
import rasterio.mask as riom
import rasterio.warp as riow
from rasterio.warp import Resampling
from rasterio import MemoryFile, Affine
from contextlib import contextmanager
import geopandas as gpd
import fiona as fio
import numpy as np
import swisssmartfarming.utils.ssf_functions as ssf
from IPython import embed

class DataInterface:
    def __init__(self):
        self.dates = []
        self.datasets_names = []
        self.datasets = {}
        self.shapefile_name = None
        self.shapefile = None
        self.transforms = {}
        self.datasets_shape = {}
        self.datasets_in_memory = {}
        self.resolutions = {}
        self.crs = {}
        self.data_mask = None
        self.no_data_val = -10000
#-------------------------------------------------------------------------------

    def add_dataset(self, dataset):
        """Adds a dataset.

        Adds a dataset, updates the dates attribute and defines its
        'dataset_name' based on its date and on the type of band.

        Parameters
        ----------
        dataset: str
            full-path to the datasets
        """

        date = dataset.split("/")[-1].split("_")[1]
        if date not in self.dates:
            self.dates.append(date)
            self.dates.sort()

        band = dataset.split("/")[-1].split("_")[-2]
        dataset_name = date + "_" + band

        if dataset_name not in self.datasets_names:
            self.datasets_names.append(dataset_name)
            self.datasets_names.sort()

        source = rio.open(dataset)
        self.datasets[dataset_name] = source
        self.transforms[dataset_name] = source.transform
        self.resolutions[dataset_name] = (source.transform[0],
            -source.transform[4])
        self.crs[dataset_name] = source.crs
        self.datasets_shape[dataset_name] = (source.height, source.width)
#-------------------------------------------------------------------------------

    def add_shapefile(self, shapefile):
        """Adds a shapefile.

        The shapefile delimits the field which will be analyzed. It can be then
        used in order to clip the dataset.

        Parameters
        ----------
        shapefile: str
            full-path to the shapefile
        """

        self.shapefile = gpd.read_file(shapefile)
        self.shapefile_name = shapefile.split("/")[-1].split(".")[0]
#-------------------------------------------------------------------------------

    def crop_dataset(self, dataset_name):
        """Crops the specified dataset using the previously loaded shapefile

        Parameters
        ----------
        dataset_name: str
            name of the dataset to crop
        """

        try:
            dataset = self.datasets[dataset_name]
        except KeyError as e:
            raise type(e)(str(e) + ' not in datasets')
        else:
            try:
                shapes = [feature["geometry"]
                    for _, feature in self.shapefile.iterrows()]
            except AttributeError:
                raise Exception("Shapefile has not been added yet.")
            else:
                data, transform = riom.mask(dataset, shapes, crop=True)
                self.transforms[dataset_name] = transform
                self.datasets_shape[dataset_name] = data.shape

                profile = self.datasets[dataset_name].profile
                profile.update(transform=transform, driver='GTiff',
                    height=data.shape[1], width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[dataset_name] = memfile.open()
#-------------------------------------------------------------------------------

    def align_datasets(self, ref_dataset):
        """Aligns all added datasets to a reference dataset.

        Resamples all the added datasets using the specified one as reference in
        order to obtain a perfect pixel-matching.

        Parameters
        ----------
        ref_dataset: str
            name of the dataset that should be used as reference
        """

        out_shape = self.datasets_shape[ref_dataset]
        self.data_mask = np.squeeze(self.datasets[ref_dataset].read()
            != self.no_data_val)

        for dataset_name in self.datasets_names:
            if not dataset_name.startswith(ref_dataset.split("_")[0]):
                dataset = self.datasets[dataset_name]
                data = dataset.read(out_shape=out_shape,
                    resampling=Resampling.bilinear)

                self.datasets_shape[dataset_name] = data.shape
                self.transforms[dataset_name] = self.transforms[ref_dataset]
                profile = self.datasets[dataset_name].profile
                profile.update(transform=self.transforms[ref_dataset],
                    driver='GTiff', height=data.shape[1], width=data.shape[2])

                with MemoryFile() as memfile:
                    with memfile.open(**profile) as dataset:
                        dataset.write(data)
                        del data

                    self.datasets[dataset_name] = memfile.open()
#-------------------------------------------------------------------------------

    def load_dataset_to_memory(self, dataset_name):
        """Loads the specified dataset to memory.

        Parameters
        ----------
        dataset_name: str
            name of the dataset that should be loaded
        """
        try:
            data = self.datasets[dataset_name].read()
        except KeyError as e:
            raise type(e)(str(e) + ' not in datasets')
        else:
            self.datasets_in_memory[dataset_name] = data
#-------------------------------------------------------------------------------


    def ndvi(self, date):
        """Computes the NDVI-map for the specified date.

        Parameters
        ----------
        date: str
            date at which the NDVI-map should be computed. The date has to be
            given under the format "yyyymmdd"

        Returns
        -------
        numpy.ndarray
            raster containing the NDVI-map for the specified date
        """
        r_name = date + "_red"
        nir_name = date + "_nir"
        try:
            red = np.squeeze(self.datasets[r_name].read())
        except KeyError as e:
            raise type(e)("'red' band for the specified date has not been added yet.")
        try:
            nir = np.squeeze(self.datasets[nir_name].read())
        except KeyError as e:
            raise type(e)("'nir' band for the specified date has not been added yet.")
        else:
            return ssf.ndvi(nir, red)
